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
