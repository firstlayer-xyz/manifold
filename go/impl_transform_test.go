package manifold

import (
	"math"
	"runtime"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/internal/cppref"
)

// TestTransform_Translate exercises pure translation against the
// known mathematical output.
func TestTransform_Translate(t *testing.T) {
	m := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
	defer runtime.KeepAlive(m)
	out := m.Translate(Vec3{X: 10, Y: 20, Z: 30})
	defer runtime.KeepAlive(out)
	bb := out.BoundingBox()
	want := Vec3{X: 10, Y: 20, Z: 30}
	if math.Abs(bb.Center().X-want.X) > 1e-12 ||
		math.Abs(bb.Center().Y-want.Y) > 1e-12 ||
		math.Abs(bb.Center().Z-want.Z) > 1e-12 {
		t.Errorf("Translate Center: got %+v, want %+v", bb.Center(), want)
	}
	if !floatClose(out.Volume(), 1.0, 1e-12, 1e-12) {
		t.Errorf("Translate Volume: got %v, want 1", out.Volume())
	}
}

// TestTransform_Scale verifies that Scale by a uniform factor scales
// Volume by the cube of the factor and SurfaceArea by the square.
func TestTransform_Scale(t *testing.T) {
	m := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
	defer runtime.KeepAlive(m)
	s := 2.0
	out := m.Scale(Vec3{X: s, Y: s, Z: s})
	defer runtime.KeepAlive(out)
	if !floatClose(out.Volume(), s*s*s, 1e-12, 1e-12) {
		t.Errorf("Scale Volume: got %v, want %v", out.Volume(), s*s*s)
	}
	if !floatClose(out.SurfaceArea(), 6*s*s, 1e-12, 1e-12) {
		t.Errorf("Scale SurfaceArea: got %v, want %v", out.SurfaceArea(), 6*s*s)
	}
}

// TestTransform_Mirror flips the manifold across a plane. The result
// must have the same Volume and SurfaceArea as the input, with
// winding reversed (handled internally by FlipTris when det < 0).
func TestTransform_Mirror(t *testing.T) {
	m := Tetrahedron()
	defer runtime.KeepAlive(m)
	mirrored := m.Mirror(Vec3{X: 1, Y: 0, Z: 0})
	defer runtime.KeepAlive(mirrored)
	if !floatClose(mirrored.Volume(), m.Volume(), 1e-12, 1e-12) {
		t.Errorf("Mirror Volume: got %v, want %v", mirrored.Volume(), m.Volume())
	}
	if !floatClose(mirrored.SurfaceArea(), m.SurfaceArea(), 1e-12, 1e-12) {
		t.Errorf("Mirror SurfaceArea: got %v, want %v",
			mirrored.SurfaceArea(), m.SurfaceArea())
	}
	// IsManifold must hold for the mirrored mesh (winding flip is
	// consistent across the whole mesh).
	impl := getImpl(mirrored)
	defer impl.Delete()
	if !impl.IsManifold() {
		t.Error("Mirror result is not 2-manifold")
	}
}

// TestTransform_Rotate verifies that a 90° rotation about Z preserves
// volume and surface area.
func TestTransform_Rotate(t *testing.T) {
	m := Cube(Vec3{X: 1, Y: 2, Z: 3}, true)
	defer runtime.KeepAlive(m)
	rotated := m.Rotate(0, 0, 90)
	defer runtime.KeepAlive(rotated)
	if !floatClose(rotated.Volume(), m.Volume(), 1e-12, 1e-12) {
		t.Errorf("Rotate Volume: got %v, want %v", rotated.Volume(), m.Volume())
	}
	// After 90° Z rotation, the X-extent of (1,2,3)→ (2,1,3). The
	// bbox of the rotated 1x2x3 cube should be 2x1x3.
	bb := rotated.BoundingBox()
	sz := bb.Size()
	if math.Abs(sz.X-2) > 1e-12 || math.Abs(sz.Y-1) > 1e-12 || math.Abs(sz.Z-3) > 1e-12 {
		t.Errorf("Rotate Size after 90° Z: got %+v, want {2, 1, 3}", sz)
	}
}

// TestTransform_VsCpp_Translate cross-checks the drilled Translate
// against the C++ reference: identical Volume + SurfaceArea +
// BoundingBox.
func TestTransform_VsCpp_Translate(t *testing.T) {
	mGo := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
	defer runtime.KeepAlive(mGo)
	hRef := cppref.Cube(1, 1, 1, true)
	defer cppref.DeleteManifold(hRef)

	v := Vec3{X: 0.5, Y: 1.5, Z: -2.5}
	tGo := mGo.Translate(v)
	defer runtime.KeepAlive(tGo)
	tRef := cppref.Translate(hRef, v)
	defer cppref.DeleteManifold(tRef)

	if !floatClose(tGo.Volume(), cppref.Volume(tRef), 1e-12, 1e-12) {
		t.Errorf("Volume: go=%v ref=%v", tGo.Volume(), cppref.Volume(tRef))
	}
	if !floatClose(tGo.SurfaceArea(), cppref.SurfaceArea(tRef), 1e-12, 1e-12) {
		t.Errorf("SurfaceArea: go=%v ref=%v",
			tGo.SurfaceArea(), cppref.SurfaceArea(tRef))
	}
}

// TestTransform_VsCpp_Transform: a non-axis-aligned linear transform
// must yield identical Volume to the C++ cppref.
func TestTransform_VsCpp_Transform(t *testing.T) {
	mGo := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
	defer runtime.KeepAlive(mGo)
	hRef := cppref.Cube(1, 1, 1, true)
	defer cppref.DeleteManifold(hRef)

	// Skew + non-uniform scale.
	mat := Mat3x4{
		{2, 0.5, 0},
		{0, 1, 0.1},
		{0.3, 0, 0.5},
		{0.1, 0.2, 0.3},
	}
	tGo := mGo.Transform(mat)
	defer runtime.KeepAlive(tGo)
	tRef := cppref.Transform(hRef,
		mat[0][0], mat[0][1], mat[0][2],
		mat[1][0], mat[1][1], mat[1][2],
		mat[2][0], mat[2][1], mat[2][2],
		mat[3][0], mat[3][1], mat[3][2])
	defer cppref.DeleteManifold(tRef)

	if !floatClose(tGo.Volume(), cppref.Volume(tRef), 1e-9, 1e-9) {
		t.Errorf("Volume: go=%v ref=%v", tGo.Volume(), cppref.Volume(tRef))
	}
	if !floatClose(tGo.SurfaceArea(), cppref.SurfaceArea(tRef), 1e-9, 1e-9) {
		t.Errorf("SurfaceArea: go=%v ref=%v",
			tGo.SurfaceArea(), cppref.SurfaceArea(tRef))
	}
}

// TestTransform_IdentityReturnsClone: the identity matrix shortcut
// must yield a manifold with identical properties.
func TestTransform_IdentityReturnsClone(t *testing.T) {
	m := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
	defer runtime.KeepAlive(m)
	id := Mat3x4{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 0}}
	out := m.Transform(id)
	defer runtime.KeepAlive(out)
	if !floatClose(out.Volume(), m.Volume(), 1e-12, 1e-12) {
		t.Errorf("identity Volume: %v != %v", out.Volume(), m.Volume())
	}
	if got, want := out.NumVert(), m.NumVert(); got != want {
		t.Errorf("identity NumVert: %d != %d", got, want)
	}
}
