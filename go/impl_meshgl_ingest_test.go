package manifold

import (
	"math"
	"runtime"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/internal/cppref"
)

// pureBridgeIngestGL64 ingests via the C++ bridge directly — the reference
// oracle for the native ingest drill.
func pureBridgeIngestGL64(m MeshGL64) *Manifold {
	return wrap(cppref.ManifoldFromMeshGL64(
		m.NumProp,
		m.VertProperties, m.TriVerts,
		m.MergeFromVert, m.MergeToVert,
		m.RunIndex, m.RunOriginalID, m.RunTransform, m.RunFlags,
		m.FaceID, m.HalfedgeTangent,
		m.Tolerance,
	))
}

func cloneMeshGL64(m MeshGL64) MeshGL64 {
	c := m
	c.VertProperties = append([]float64(nil), m.VertProperties...)
	c.TriVerts = append([]uint64(nil), m.TriVerts...)
	c.MergeFromVert = append([]uint64(nil), m.MergeFromVert...)
	c.MergeToVert = append([]uint64(nil), m.MergeToVert...)
	c.RunIndex = append([]uint64(nil), m.RunIndex...)
	c.RunOriginalID = append([]uint32(nil), m.RunOriginalID...)
	c.RunTransform = append([]float64(nil), m.RunTransform...)
	c.RunFlags = append([]uint8(nil), m.RunFlags...)
	c.FaceID = append([]uint64(nil), m.FaceID...)
	c.HalfedgeTangent = append([]float64(nil), m.HalfedgeTangent...)
	return c
}

// TestMeshGLIngest_VsCpp is the oracle for the MeshGL ingest drill: it
// compares the native-fronted NewManifoldFromMeshGL64 against the pure C++
// bridge ingest. Valid inputs must produce identical geometry; every
// error-cascade input must produce the identical Error status. (At this
// increment only the validation cascade is native; valid inputs still
// defer to the bridge body, so geometry parity is by construction — what
// this locks in is that the native cascade neither wrongly rejects valid
// meshes nor reports the wrong Error for malformed ones.)
func TestMeshGLIngest_VsCpp(t *testing.T) {
	// Valid bases via the (drilled) export path.
	cube := Cube(Vec3{X: 1, Y: 1, Z: 1}, false)
	defer runtime.KeepAlive(cube)
	base := cube.GetMeshGL64(-1)
	numTri := len(base.TriVerts) / 3

	cases := []struct {
		name string
		mk   func() MeshGL64
	}{
		{"valid_cube", func() MeshGL64 { return cloneMeshGL64(base) }},
		{"valid_sphere", func() MeshGL64 { s := Sphere(1, 16); defer runtime.KeepAlive(s); return s.GetMeshGL64(-1) }},
		{"empty", func() MeshGL64 { return MeshGL64{NumProp: 3} }},
		{"tooSmall_tris", func() MeshGL64 { c := cloneMeshGL64(base); c.TriVerts = c.TriVerts[:9]; return c }},
		{"numProp_lt3", func() MeshGL64 { c := cloneMeshGL64(base); c.NumProp = 2; return c }},
		{"merge_mismatch", func() MeshGL64 {
			c := cloneMeshGL64(base)
			c.MergeFromVert = []uint64{0}
			c.MergeToVert = nil
			return c
		}},
		{"transform_wrong", func() MeshGL64 { c := cloneMeshGL64(base); c.RunTransform = []float64{1, 2, 3}; return c }},
		{"runindex_wrong", func() MeshGL64 {
			c := cloneMeshGL64(base)
			c.RunOriginalID = []uint32{5, 6}
			c.RunIndex = []uint64{0}
			return c
		}},
		{"faceid_wrong", func() MeshGL64 { c := cloneMeshGL64(base); c.FaceID = []uint64{0}; return c }},
		{"nonfinite_vert", func() MeshGL64 { c := cloneMeshGL64(base); c.VertProperties[0] = math.NaN(); return c }},
		{"nonfinite_transform", func() MeshGL64 {
			c := cloneMeshGL64(base)
			c.RunOriginalID = []uint32{5}
			c.RunTransform = make([]float64, 12)
			for i := range c.RunTransform {
				c.RunTransform[i] = math.NaN()
			}
			return c
		}},
		{"nonfinite_tangent", func() MeshGL64 {
			c := cloneMeshGL64(base)
			c.HalfedgeTangent = []float64{math.NaN(), 0, 0, 0}
			return c
		}},
		// extra property column, no merge -> exercises the native
		// dual-stride vert/prop split (NumProp 3 -> 4).
		{"props_no_merge", func() MeshGL64 {
			c := cloneMeshGL64(base)
			c.MergeFromVert, c.MergeToVert = nil, nil
			nv := len(c.VertProperties) / 3
			np := make([]float64, nv*4)
			for i := 0; i < nv; i++ {
				np[4*i+0] = c.VertProperties[3*i+0]
				np[4*i+1] = c.VertProperties[3*i+1]
				np[4*i+2] = c.VertProperties[3*i+2]
				np[4*i+3] = float64(i) * 0.5 // arbitrary finite extra prop
			}
			c.VertProperties = np
			c.NumProp = 4
			return c
		}},
		// cube with per-face normals: extra props (NumProp=6) AND a vertex
		// merge -> needsPropMap, exercising the two-arg CreateHalfedges.
		{"cube_normals_needsPropMap", func() MeshGL64 {
			cn := Cube(Vec3{X: 1, Y: 1, Z: 1}, false).CalculateNormals(0, 60)
			defer runtime.KeepAlive(cn)
			return cn.GetMeshGL64(0)
		}},
		// malformed index with high bits set but a valid low-32: C++ casts
		// the uint64 index to uint32 before the bounds check, so both impls
		// must truncate to the original valid index (not VertexOutOfBounds).
		{"highbit_index_truncates", func() MeshGL64 {
			c := cloneMeshGL64(base)
			c.TriVerts[0] |= uint64(1) << 32
			return c
		}},
	}

	_ = numTri
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			m := tc.mk()
			goM := NewManifoldFromMeshGL64(m)
			defer runtime.KeepAlive(goM)
			refM := pureBridgeIngestGL64(m)
			defer runtime.KeepAlive(refM)

			if g, r := goM.Status(), refM.Status(); g != r {
				t.Fatalf("Status: go=%d ref=%d", g, r)
			}
			if refM.Status() == NoError && !refM.IsEmpty() {
				assertSameManifold(t, goM, refM)
			}
		})
	}
}

// TestMeshGLIngest_Float32_VsCpp exercises the MeshGL (float32) ingest path,
// which sets useSingle=true in SetEpsilon — the single most error-prone
// point per the scoping analysis (a wrong flag silently shifts tolerance_).
func TestMeshGLIngest_Float32_VsCpp(t *testing.T) {
	for _, mk := range []func() *Manifold{
		func() *Manifold { return Cube(Vec3{X: 1, Y: 1, Z: 1}, false) },
		func() *Manifold { return Sphere(1, 16) },
	} {
		src := mk()
		defer runtime.KeepAlive(src)
		m := src.GetMeshGL(-1)
		goM := NewManifoldFromMeshGL(m)
		defer runtime.KeepAlive(goM)
		refM := wrap(cppref.ManifoldFromMeshGL(
			m.NumProp,
			m.VertProperties, m.TriVerts,
			m.MergeFromVert, m.MergeToVert,
			m.RunIndex, m.RunOriginalID, m.RunTransform, m.RunFlags,
			m.FaceID, m.HalfedgeTangent,
			m.Tolerance,
		))
		defer runtime.KeepAlive(refM)
		if g, r := goM.Status(), refM.Status(); g != r {
			t.Fatalf("Status: go=%d ref=%d", g, r)
		}
		assertSameManifold(t, goM, refM)
	}
}
