package manifold

import (
	"math"
	"sort"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/boolean"
	"github.com/firstlayer-xyz/manifold/go/internal/collider"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/reference"
)

// TestRayCast_VsCpp validates the entire native Boolean3 kernel cascade
// (Shadow01 -> Kernel02 -> Kernel11 -> Kernel12) end-to-end via RayCast against
// the C++ bridge. The cascade is pure arithmetic (no transcendentals), so the
// hits should match essentially bit-exactly.
func TestRayCast_VsCpp(t *testing.T) {
	cases := []struct {
		name string
		mk   func() *Manifold
	}{
		{"cube", func() *Manifold { return Cube(Vec3{X: 2, Y: 2, Z: 2}, true) }},
		{"sphere", func() *Manifold { return Sphere(1, 32) }},
	}
	rays := []struct{ o, e geom.Vec3 }{
		{geom.Vec3{X: -5, Y: 0, Z: 0}, geom.Vec3{X: 5, Y: 0, Z: 0}},           // through center, +x
		{geom.Vec3{X: -5, Y: 0.3, Z: 0.2}, geom.Vec3{X: 5, Y: 0.3, Z: 0.2}},   // off-center +x
		{geom.Vec3{X: 0, Y: -5, Z: 0.1}, geom.Vec3{X: 0, Y: 5, Z: 0.1}},       // +y
		{geom.Vec3{X: 0.1, Y: 0.2, Z: -5}, geom.Vec3{X: 0.1, Y: 0.2, Z: 5}},   // +z
		{geom.Vec3{X: 0, Y: 0, Z: 0}, geom.Vec3{X: 5, Y: 0.1, Z: 0.1}},        // from inside outward
		{geom.Vec3{X: -3, Y: -2.7, Z: -2.4}, geom.Vec3{X: 3, Y: 2.6, Z: 2.3}}, // skew diagonal
		{geom.Vec3{X: 10, Y: 10, Z: 10}, geom.Vec3{X: 20, Y: 20, Z: 20}},      // miss
	}

	canon := func(h []boolean.RayHit) []boolean.RayHit {
		out := append([]boolean.RayHit(nil), h...)
		sort.Slice(out, func(i, j int) bool {
			if out[i].FaceID != out[j].FaceID {
				return out[i].FaceID < out[j].FaceID
			}
			return out[i].Distance < out[j].Distance
		})
		return out
	}

	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			m := tc.mk()
			view := getImpl(m)
			defer view.Delete()
			faceBox, faceMorton := view.GetFaceBoxMorton()
			col := collider.New(faceBox, faceMorton)

			// C++ oracle: seal the native mesh into a bridge handle and read its Impl.
			rh := m.refHandle()
			defer reference.DeleteManifold(rh)
			bimpl := bridge.GetImpl(rh)
			defer bimpl.Delete()

			for ri, r := range rays {
				native := canon(boolean.RayCast(view.Verts(), view.VertNormals(),
					view.FaceNormals(), view.HalfedgeStarts(), view.HalfedgePairs(),
					col, r.o, r.e))
				refRaw := bimpl.RayCast(r.o, r.e)
				ref := make([]boolean.RayHit, len(refRaw))
				for i, h := range refRaw {
					ref[i] = boolean.RayHit{FaceID: h.FaceID, Distance: h.Distance, Position: h.Position, Normal: h.Normal}
				}
				ref = canon(ref)

				if len(native) != len(ref) {
					t.Fatalf("ray %d: native %d hits != bridge %d", ri, len(native), len(ref))
				}
				for i := range native {
					n, b := native[i], ref[i]
					if n.FaceID != b.FaceID || math.Abs(n.Distance-b.Distance) > 1e-12 ||
						math.Abs(n.Position.X-b.Position.X) > 1e-12 ||
						math.Abs(n.Position.Y-b.Position.Y) > 1e-12 ||
						math.Abs(n.Position.Z-b.Position.Z) > 1e-12 {
						t.Fatalf("ray %d hit %d: native %+v != bridge %+v", ri, i, n, b)
					}
				}
			}
		})
	}
}
