package manifold

import (
	"math"
	"runtime"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/internal/cppref"
)

// TestSetNormals_VsCpp_Cube exercises the multi-normal split path —
// a cube has 90° dihedrals between every adjacent face, so every
// vertex (with minSharpAngle < 90) becomes a multi-normal vert and
// the per-corner propVert is duplicated three ways. Compare the Go
// and C++ outputs: NumProp, NumVert, NumPropVert, total vert count
// of the per-face normal directions.
func TestSetNormals_VsCpp_Cube(t *testing.T) {
	for _, minSharp := range []float64{60.0, 30.0} {
		t.Run("", func(t *testing.T) {
			mGo := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
			defer runtime.KeepAlive(mGo)
			hRef := cppref.Cube(1, 1, 1, true)
			defer cppref.DeleteManifold(hRef)

			nGo := mGo.CalculateNormals(0, minSharp)
			defer runtime.KeepAlive(nGo)
			nRef := cppref.CalculateNormals(hRef, 0, minSharp)
			defer cppref.DeleteManifold(nRef)

			if got, want := nGo.NumProp(), cppref.NumProp(nRef); got != want {
				t.Errorf("NumProp: go=%d ref=%d", got, want)
			}
			if !floatClose(nGo.Volume(), cppref.Volume(nRef), 1e-9, 1e-9) {
				t.Errorf("Volume: go=%v ref=%v",
					nGo.Volume(), cppref.Volume(nRef))
			}
			if !floatClose(nGo.SurfaceArea(), cppref.SurfaceArea(nRef), 1e-9, 1e-9) {
				t.Errorf("SurfaceArea: go=%v ref=%v",
					nGo.SurfaceArea(), cppref.SurfaceArea(nRef))
			}
		})
	}
}

// TestSetNormals_VsCpp_Sphere exercises the smooth-vert path: a
// sphere has no sharp edges (every dihedral is <60°), so every vert
// takes a single smooth normal. The Go and C++ outputs must agree on
// NumProp, Volume, SurfaceArea.
func TestSetNormals_VsCpp_Sphere(t *testing.T) {
	mGo := Sphere(1.0, 24)
	defer runtime.KeepAlive(mGo)
	hRef := cppref.Sphere(1.0, 24)
	defer cppref.DeleteManifold(hRef)

	nGo := mGo.CalculateNormals(0, 60)
	defer runtime.KeepAlive(nGo)
	nRef := cppref.CalculateNormals(hRef, 0, 60)
	defer cppref.DeleteManifold(nRef)

	if got, want := nGo.NumProp(), cppref.NumProp(nRef); got != want {
		t.Errorf("NumProp: go=%d ref=%d", got, want)
	}
	if !floatClose(nGo.Volume(), cppref.Volume(nRef), 1e-9, 1e-9) {
		t.Errorf("Volume: go=%v ref=%v", nGo.Volume(), cppref.Volume(nRef))
	}
	if !floatClose(nGo.SurfaceArea(), cppref.SurfaceArea(nRef), 1e-9, 1e-9) {
		t.Errorf("SurfaceArea: go=%v ref=%v", nGo.SurfaceArea(), cppref.SurfaceArea(nRef))
	}
}

// TestKMinSharpAngle_MatchesCpp guards the faithfulness fix for the dihedral floor:
// src/smoothing.cpp:44 is `constexpr double kMinSharpAngle = 1e-4` (degrees). A prior
// transcription had 5.0, which silently reclassified all near-coplanar edges (1e-4..5
// deg) as sharp. Differential tests missed it because they only pass minSharpAngle
// values (30/60 deg) far above both, where the clamp is identical.
func TestKMinSharpAngle_MatchesCpp(t *testing.T) {
	if kMinSharpAngle != 1e-4 {
		t.Errorf("kMinSharpAngle = %v, want 1e-4 (must mirror src/smoothing.cpp:44)", kMinSharpAngle)
	}
}

// TestSetNormals_VsCpp_SmallMinSharpAngle exercises the (1e-4, 5) degree gap that the
// kMinSharpAngle bug lived in. A 128-segment sphere has equatorial dihedrals ~2.8 deg;
// with minSharpAngle = 1 deg those edges are SHARP (split propVerts), but a 5-degree
// floor would clamp minSharpAngle up to 5 and treat them as smooth — producing far
// fewer propVerts than C++. So NumPropVert is the discriminator and must match the oracle.
func TestSetNormals_VsCpp_SmallMinSharpAngle(t *testing.T) {
	for _, minSharp := range []float64{0.5, 1.0, 2.0} {
		t.Run("", func(t *testing.T) {
			mGo := Sphere(1.0, 128)
			defer runtime.KeepAlive(mGo)
			hRef := cppref.Sphere(1.0, 128)
			defer cppref.DeleteManifold(hRef)

			nGo := mGo.CalculateNormals(0, minSharp)
			defer runtime.KeepAlive(nGo)
			nRef := cppref.CalculateNormals(hRef, 0, minSharp)
			defer cppref.DeleteManifold(nRef)

			if got, want := nGo.NumPropVert(), cppref.NumPropVert(nRef); got != want {
				t.Errorf("NumPropVert: go=%d ref=%d (minSharp=%v) — kMinSharpAngle floor regressed?", got, want, minSharp)
			}
			// Sanity: the gap angle really does split propVerts (guards against the
			// test silently passing if both sides took the all-smooth path).
			if nGo.NumPropVert() <= nGo.NumVert() {
				t.Errorf("expected sharp-edge splits (NumPropVert > NumVert) at minSharp=%v; got NumPropVert=%d NumVert=%d",
					minSharp, nGo.NumPropVert(), nGo.NumVert())
			}
		})
	}
}

// TestSetNormals_PerVertNormalLength: every stored normal must be a
// unit vector (or zero, for degenerate verts). This is a stronger
// check than just Volume/SurfaceArea matching.
func TestSetNormals_PerVertNormalLength(t *testing.T) {
	m := Sphere(1.0, 24)
	defer runtime.KeepAlive(m)
	n := m.CalculateNormals(0, 60)
	defer runtime.KeepAlive(n)
	mesh := n.GetMeshGL(-1)
	numProp := mesh.NumProp
	if numProp < 6 {
		t.Fatalf("expected NumProp >= 6 (xyz + normal), got %d", numProp)
	}
	for v := 0; v < len(mesh.VertProperties); v += numProp {
		nx := float64(mesh.VertProperties[v+3])
		ny := float64(mesh.VertProperties[v+4])
		nz := float64(mesh.VertProperties[v+5])
		mag := math.Sqrt(nx*nx + ny*ny + nz*nz)
		if mag < 0.99 || mag > 1.01 {
			t.Errorf("vertex %d normal not unit-length: (%v,%v,%v) |%v|",
				v/numProp, nx, ny, nz, mag)
			break
		}
	}
}
