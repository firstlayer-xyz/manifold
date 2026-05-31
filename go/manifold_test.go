package manifold

import (
	"math"
	"runtime"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/handle"
	"github.com/firstlayer-xyz/manifold/go/reference"
)

// TestNumVert_Tetrahedron_Differential constructs a tetrahedron via the C++
// reference path, then asserts the Go-side NumVert returns the same value as
// the reference-side NumVert (and that both return the expected 4).
func TestNumVert_Tetrahedron_Differential(t *testing.T) {
	h := reference.Tetrahedron()
	defer reference.DeleteManifold(h)

	m := &Manifold{h: h}

	got := m.NumVert()
	want := reference.NumVert(h)

	if got != want {
		t.Errorf("NumVert mismatch: go=%d ref=%d", got, want)
	}
	if got != 4 {
		t.Errorf("tetrahedron NumVert: got %d, want 4", got)
	}
}

// TestTranslate_Tetrahedron_Differential checks Translate via bounding box
// AND volume (volume catches inverted-orientation bugs that the bbox check
// would silently accept).
func TestTranslate_Tetrahedron_Differential(t *testing.T) {
	v := Vec3{X: 2, Y: 3, Z: 4}

	hOrig := reference.Tetrahedron()
	defer reference.DeleteManifold(hOrig)

	m := &Manifold{h: hOrig}
	mGo := m.Translate(v)
	defer runtime.KeepAlive(mGo)

	hRef := reference.Translate(hOrig, v)
	defer reference.DeleteManifold(hRef)

	assertSameBoundingBox(t, mGo.h, hRef, 1e-12)
	if !floatClose(mGo.Volume(), reference.Volume(hRef), 1e-12, 1e-12) {
		t.Errorf("translated Volume: go=%v ref=%v", mGo.Volume(), reference.Volume(hRef))
	}
}

// TestScale_Tetrahedron_Differential checks Scale via bounding box AND
// volume.
func TestScale_Tetrahedron_Differential(t *testing.T) {
	v := Vec3{X: 2, Y: 3, Z: 4}

	hOrig := reference.Tetrahedron()
	defer reference.DeleteManifold(hOrig)

	m := &Manifold{h: hOrig}
	mGo := m.Scale(v)
	defer runtime.KeepAlive(mGo)

	hRef := reference.Scale(hOrig, v)
	defer reference.DeleteManifold(hRef)

	assertSameBoundingBox(t, mGo.h, hRef, 1e-12)
	if !floatClose(mGo.Volume(), reference.Volume(hRef), 1e-12, 1e-12) {
		t.Errorf("scaled Volume: go=%v ref=%v", mGo.Volume(), reference.Volume(hRef))
	}
}

// TestBoundingBox_Tetrahedron_Differential verifies the Go-side BoundingBox
// method returns the same box as the C++ reference for a tetrahedron and
// for a translated tetrahedron (to ensure non-trivial values).
func TestBoundingBox_Tetrahedron_Differential(t *testing.T) {
	cases := []struct {
		name      string
		translate Vec3
	}{
		{"at origin", Vec3{}},
		{"translated", Vec3{X: 2, Y: -3, Z: 4.5}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			hOrig := reference.Tetrahedron()
			defer reference.DeleteManifold(hOrig)

			m := (&Manifold{h: hOrig}).Translate(tc.translate)
			defer runtime.KeepAlive(m)
			hRef := reference.Translate(hOrig, tc.translate)
			defer reference.DeleteManifold(hRef)

			gotBox := m.BoundingBox()
			wantBox := refBoundingBox(hRef)

			const eps = 1e-12
			if !vec3Close(gotBox.Min, wantBox.Min, eps) || !vec3Close(gotBox.Max, wantBox.Max, eps) {
				t.Errorf("BoundingBox mismatch:\n  go:  %+v\n  ref: %+v", gotBox, wantBox)
			}
		})
	}
}

// refBoundingBox wraps reference.BoundingBox into a public Box for symmetry
// with the Go-side accessor.
func refBoundingBox(h *handle.Manifold) Box {
	min, max := reference.BoundingBox(h)
	return Box{Min: min, Max: max}
}

// TestMirror_Tetrahedron_Differential checks Mirror across several planes
// and against the zero-normal edge case (which should return an empty
// Manifold matching the C++ reference behavior).
func TestMirror_Tetrahedron_Differential(t *testing.T) {
	cases := []struct {
		name   string
		normal Vec3
		eps    float64
	}{
		{"+X plane", Vec3{X: 1, Y: 0, Z: 0}, 1e-12},
		{"+Y plane", Vec3{X: 0, Y: 1, Z: 0}, 1e-12},
		{"+Z plane", Vec3{X: 0, Y: 0, Z: 1}, 1e-12},
		{"diagonal unit", Vec3{X: 1, Y: 1, Z: 1}, 1e-10},        // requires normalization
		{"diagonal non-unit", Vec3{X: 3, Y: -4, Z: 12}, 1e-10}, // length 13, requires normalization
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			hOrig := reference.Tetrahedron()
			defer reference.DeleteManifold(hOrig)

			m := &Manifold{h: hOrig}
			mGo := m.Mirror(tc.normal)
			defer runtime.KeepAlive(mGo)

			hRef := reference.Mirror(hOrig, tc.normal)
			defer reference.DeleteManifold(hRef)

			assertSameBoundingBox(t, mGo.h, hRef, tc.eps)
			// Volume catches the triangle-orientation flip: a forgotten
			// flip would invert the manifold and give negative volume.
			if !floatClose(mGo.Volume(), reference.Volume(hRef), tc.eps, tc.eps) {
				t.Errorf("mirrored Volume: go=%v ref=%v",
					mGo.Volume(), reference.Volume(hRef))
			}
		})
	}

	t.Run("zero normal returns empty", func(t *testing.T) {
		hOrig := reference.Tetrahedron()
		defer reference.DeleteManifold(hOrig)

		m := &Manifold{h: hOrig}
		mGo := m.Mirror(Vec3{})
		defer runtime.KeepAlive(mGo)

		hRef := reference.Mirror(hOrig, Vec3{})
		defer reference.DeleteManifold(hRef)

		if got, want := mGo.NumVert(), reference.NumVert(hRef); got != want {
			t.Errorf("zero-normal NumVert: go=%d ref=%d (expected both 0)", got, want)
		}
		if mGo.NumVert() != 0 {
			t.Errorf("zero-normal mirror should be empty, got NumVert=%d", mGo.NumVert())
		}
	})
}

// TestEmpty_AccessorEdgeCases exercises the Go-native accessors on an empty
// Manifold (obtained via Mirror with zero normal). All counts should be 0
// and Volume/SurfaceArea should be 0; BoundingBox should be the C++
// default (+inf min, -inf max) so it matches the reference exactly.
func TestEmpty_AccessorEdgeCases(t *testing.T) {
	hOrig := reference.Tetrahedron()
	defer reference.DeleteManifold(hOrig)

	m := (&Manifold{h: hOrig}).Mirror(Vec3{}) // zero normal -> empty
	defer runtime.KeepAlive(m)
	hRef := reference.Mirror(hOrig, Vec3{})
	defer reference.DeleteManifold(hRef)

	if !m.IsEmpty() {
		t.Errorf("IsEmpty should be true for empty manifold")
	}
	if m.NumVert() != 0 || m.NumTri() != 0 || m.NumEdge() != 0 {
		t.Errorf("empty counts: vert=%d tri=%d edge=%d", m.NumVert(), m.NumTri(), m.NumEdge())
	}
	if v := m.Volume(); v != 0 {
		t.Errorf("Volume on empty: got %v, want 0", v)
	}
	if a := m.SurfaceArea(); a != 0 {
		t.Errorf("SurfaceArea on empty: got %v, want 0", a)
	}

	// Compare BoundingBox to reference. Reference returns whatever C++
	// stores in bBox_ for an empty mesh.
	gotBox := m.BoundingBox()
	wantMin, wantMax := reference.BoundingBox(hRef)
	if gotBox.Min != wantMin || gotBox.Max != wantMax {
		t.Errorf("empty BoundingBox:\n  go:  min=%+v max=%+v\n  ref: min=%+v max=%+v",
			gotBox.Min, gotBox.Max, wantMin, wantMax)
	}
}

// TestSphere_VolumeSurfaceArea_Differential exercises the Kahan-summed
// Go-native Volume / SurfaceArea against the C++ reference on a sphere,
// where the triangle count is large enough that summation order matters.
func TestSphere_VolumeSurfaceArea_Differential(t *testing.T) {
	const radius = 1.0
	const segments = 64
	mGo := Sphere(radius, segments)
	defer runtime.KeepAlive(mGo)
	hRef := reference.Sphere(radius, segments)
	defer reference.DeleteManifold(hRef)

	if !floatClose(mGo.Volume(), reference.Volume(hRef), 1e-10, 1e-12) {
		t.Errorf("Sphere Volume: go=%v ref=%v", mGo.Volume(), reference.Volume(hRef))
	}
	if !floatClose(mGo.SurfaceArea(), reference.SurfaceArea(hRef), 1e-10, 1e-12) {
		t.Errorf("Sphere SurfaceArea: go=%v ref=%v",
			mGo.SurfaceArea(), reference.SurfaceArea(hRef))
	}
}

// TestRotate_Tetrahedron_Differential exercises the first port that does
// real numeric math in Go (Sind/Cosd + matrix multiply). Several angle
// triples cover trivial (multiple-of-90) and arbitrary cases. Looser eps
// than the affine tests because Go's math.Sin / math.Cos and C++'s differ
// in the LSBs even with degree-exact reduction.
func TestRotate_Tetrahedron_Differential(t *testing.T) {
	cases := []struct {
		name          string
		x, y, z, eps  float64
	}{
		{"identity", 0, 0, 0, 1e-12},
		{"90 about X", 90, 0, 0, 1e-12},
		{"180 about Y", 0, 180, 0, 1e-12},
		{"270 about Z", 0, 0, 270, 1e-12},
		{"all axes 45", 45, 45, 45, 1e-10},
		{"arbitrary", 17.3, -42.1, 123.7, 1e-10},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			hOrig := reference.Tetrahedron()
			defer reference.DeleteManifold(hOrig)

			m := &Manifold{h: hOrig}
			mGo := m.Rotate(tc.x, tc.y, tc.z)
			defer runtime.KeepAlive(mGo)

			hRef := reference.Rotate(hOrig, tc.x, tc.y, tc.z)
			defer reference.DeleteManifold(hRef)

			assertSameBoundingBox(t, mGo.h, hRef, tc.eps)
			// Volume is invariant under rotation; catches orientation bugs
			// that the bbox check can't see when bboxes happen to coincide.
			if !floatClose(mGo.Volume(), reference.Volume(hRef), tc.eps, tc.eps) {
				t.Errorf("rotated Volume: go=%v ref=%v", mGo.Volume(), reference.Volume(hRef))
			}
		})
	}
}

func assertSameBoundingBox(t *testing.T, got, want *handle.Manifold, eps float64) {
	t.Helper()
	minGot, maxGot := reference.BoundingBox(got)
	minWant, maxWant := reference.BoundingBox(want)
	if !vec3Close(minGot, minWant, eps) || !vec3Close(maxGot, maxWant, eps) {
		t.Errorf("bounding boxes differ:\n  go:  min=%v max=%v\n  ref: min=%v max=%v",
			minGot, maxGot, minWant, maxWant)
	}
}

func vec3Close(a, b Vec3, eps float64) bool {
	return math.Abs(a.X-b.X) <= eps && math.Abs(a.Y-b.Y) <= eps && math.Abs(a.Z-b.Z) <= eps
}

func floatClose(a, b, abs, rel float64) bool {
	d := math.Abs(a - b)
	return d <= abs || d <= rel*math.Max(math.Abs(a), math.Abs(b))
}

// TestAccessors_Tetrahedron_Differential covers the scalar/boolean
// accessors NumTri, NumEdge, IsEmpty, Volume, SurfaceArea against the C++
// reference for a tetrahedron.
func TestAccessors_Tetrahedron_Differential(t *testing.T) {
	hOrig := reference.Tetrahedron()
	defer reference.DeleteManifold(hOrig)
	m := &Manifold{h: hOrig}

	if got, want := m.NumTri(), reference.NumTri(hOrig); got != want {
		t.Errorf("NumTri: go=%d ref=%d", got, want)
	}
	if got, want := m.NumEdge(), reference.NumEdge(hOrig); got != want {
		t.Errorf("NumEdge: go=%d ref=%d", got, want)
	}
	if got, want := m.IsEmpty(), reference.IsEmpty(hOrig); got != want {
		t.Errorf("IsEmpty: go=%v ref=%v", got, want)
	}
	if got, want := m.Volume(), reference.Volume(hOrig); !floatClose(got, want, 1e-12, 1e-12) {
		t.Errorf("Volume: go=%v ref=%v", got, want)
	}
	if got, want := m.SurfaceArea(), reference.SurfaceArea(hOrig); !floatClose(got, want, 1e-12, 1e-12) {
		t.Errorf("SurfaceArea: go=%v ref=%v", got, want)
	}
}

// TestTetrahedron_GoFactory_Differential verifies that the Go-built
// tetrahedron matches the C++ reference in vert count, bbox, volume, and
// surface area. Volume in particular catches inverted-triangle bugs in
// the hardcoded triVerts list.
func TestTetrahedron_GoFactory_Differential(t *testing.T) {
	mGo := Tetrahedron()
	defer runtime.KeepAlive(mGo)
	hRef := reference.Tetrahedron()
	defer reference.DeleteManifold(hRef)

	if got, want := mGo.NumVert(), reference.NumVert(hRef); got != want {
		t.Errorf("NumVert: go=%d ref=%d", got, want)
	}
	if got, want := mGo.NumTri(), reference.NumTri(hRef); got != want {
		t.Errorf("NumTri: go=%d ref=%d", got, want)
	}
	assertSameBoundingBox(t, mGo.h, hRef, 1e-12)
	if !floatClose(mGo.Volume(), reference.Volume(hRef), 1e-12, 1e-12) {
		t.Errorf("Volume: go=%v ref=%v", mGo.Volume(), reference.Volume(hRef))
	}
	if !floatClose(mGo.SurfaceArea(), reference.SurfaceArea(hRef), 1e-12, 1e-12) {
		t.Errorf("SurfaceArea: go=%v ref=%v",
			mGo.SurfaceArea(), reference.SurfaceArea(hRef))
	}
}

// TestCube_GoFactory_Differential covers Cube in centered and
// origin-corner modes, plus the invalid-size empty branch. Volume
// catches inverted-triangle bugs in the hardcoded triVerts list.
func TestCube_GoFactory_Differential(t *testing.T) {
	cases := []struct {
		name   string
		size   Vec3
		center bool
	}{
		{"unit corner", Vec3{X: 1, Y: 1, Z: 1}, false},
		{"unit centered", Vec3{X: 1, Y: 1, Z: 1}, true},
		{"non-uniform corner", Vec3{X: 2, Y: 3, Z: 5}, false},
		{"non-uniform centered", Vec3{X: 2, Y: 3, Z: 5}, true},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			mGo := Cube(tc.size, tc.center)
			defer runtime.KeepAlive(mGo)
			hRef := reference.Cube(tc.size.X, tc.size.Y, tc.size.Z, tc.center)
			defer reference.DeleteManifold(hRef)
			assertSameBoundingBox(t, mGo.h, hRef, 1e-12)
			if !floatClose(mGo.Volume(), reference.Volume(hRef), 1e-12, 1e-12) {
				t.Errorf("Volume: go=%v ref=%v", mGo.Volume(), reference.Volume(hRef))
			}
			if got, want := mGo.NumTri(), reference.NumTri(hRef); got != want {
				t.Errorf("NumTri: go=%d ref=%d", got, want)
			}
		})
	}

	// Invalid-size paths: negative component and all-zero must return
	// an empty Manifold (matching the C++ contract).
	t.Run("negative size empty", func(t *testing.T) {
		m := Cube(Vec3{X: -1, Y: 1, Z: 1}, false)
		defer runtime.KeepAlive(m)
		if !m.IsEmpty() {
			t.Errorf("Cube with negative size should be empty, NumVert=%d", m.NumVert())
		}
	})
	t.Run("zero size empty", func(t *testing.T) {
		m := Cube(Vec3{}, false)
		defer runtime.KeepAlive(m)
		if !m.IsEmpty() {
			t.Errorf("Cube with zero size should be empty, NumVert=%d", m.NumVert())
		}
	})
}

// TestAsOriginal_Differential exercises the top-down port of AsOriginal:
// the Go body calls each inner C++ function via the bridge, matching the
// C++ implementation line for line. The resulting Manifold must agree
// with the reference on geometry, on having a non-(-1) OriginalID after
// the call (the cube starts as a primitive with originalID set, then
// gets a fresh one), and on Status == NoError.
func TestAsOriginal_Differential(t *testing.T) {
	// Use a Boolean result so the source Manifold actually has CSG history
	// to forget.
	aGo := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
	bGo := Cube(Vec3{X: 1, Y: 1, Z: 1}, true).Translate(Vec3{X: 0.5})
	defer runtime.KeepAlive(aGo)
	defer runtime.KeepAlive(bGo)
	combined := aGo.Union(bGo)
	defer runtime.KeepAlive(combined)

	aRef := reference.Cube(1, 1, 1, true)
	defer reference.DeleteManifold(aRef)
	bRefBase := reference.Cube(1, 1, 1, true)
	defer reference.DeleteManifold(bRefBase)
	bRef := reference.Translate(bRefBase, Vec3{X: 0.5})
	defer reference.DeleteManifold(bRef)
	combinedRef := reference.Union(aRef, bRef)
	defer reference.DeleteManifold(combinedRef)

	mGo := combined.AsOriginal()
	defer runtime.KeepAlive(mGo)
	mRef := reference.AsOriginal(combinedRef)
	defer reference.DeleteManifold(mRef)

	if !floatClose(mGo.Volume(), reference.Volume(mRef), 1e-9, 1e-9) {
		t.Errorf("Volume: go=%v ref=%v", mGo.Volume(), reference.Volume(mRef))
	}
	if mGo.NumVert() != reference.NumVert(mRef) {
		t.Errorf("NumVert: go=%d ref=%d", mGo.NumVert(), reference.NumVert(mRef))
	}
	if mGo.Status() != NoError {
		t.Errorf("Status: got %v, want NoError", mGo.Status())
	}
	// AsOriginal assigns a fresh non-negative original ID.
	if mGo.OriginalID() < 0 {
		t.Errorf("OriginalID: got %d, want >= 0", mGo.OriginalID())
	}
}

// TestBoolean_DirectAPI_Differential exercises the public Boolean(other,
// op) entry point (not just the Union/Difference/Intersection wrappers).
func TestBoolean_DirectAPI_Differential(t *testing.T) {
	aGo := Cube(Vec3{X: 2, Y: 2, Z: 2}, true)
	bGo := Cube(Vec3{X: 2, Y: 2, Z: 2}, true).Translate(Vec3{X: 1, Y: 1, Z: 1})
	defer runtime.KeepAlive(aGo)
	defer runtime.KeepAlive(bGo)
	aRef := reference.Cube(2, 2, 2, true)
	defer reference.DeleteManifold(aRef)
	bRefBase := reference.Cube(2, 2, 2, true)
	defer reference.DeleteManifold(bRefBase)
	bRef := reference.Translate(bRefBase, Vec3{X: 1, Y: 1, Z: 1})
	defer reference.DeleteManifold(bRef)

	for _, op := range []OpType{OpAdd, OpSubtract, OpIntersect} {
		var refOp = map[OpType]func(a, b *handle.Manifold) *handle.Manifold{
			OpAdd:       reference.Union,
			OpSubtract:  reference.Difference,
			OpIntersect: reference.Intersection,
		}[op]

		gotGo := aGo.Boolean(bGo, op)
		defer runtime.KeepAlive(gotGo)
		gotRef := refOp(aRef, bRef)
		defer reference.DeleteManifold(gotRef)
		if !floatClose(gotGo.Volume(), reference.Volume(gotRef), 1e-9, 1e-9) {
			t.Errorf("Boolean(op=%d) Volume: go=%v ref=%v",
				op, gotGo.Volume(), reference.Volume(gotRef))
		}
	}
}

// TestTrimByPlane_Differential exercises the top-down port of
// TrimByPlane, which is built from Halfspace + Intersection in Go.
func TestTrimByPlane_Differential(t *testing.T) {
	cases := []struct {
		name   string
		normal Vec3
		offset float64
	}{
		{"+X at 0", Vec3{X: 1}, 0},
		{"+Y at 0.25", Vec3{Y: 1}, 0.25},
		{"diagonal", Vec3{X: 1, Y: 1, Z: 1}, 0.5},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			mGo := Cube(Vec3{X: 2, Y: 2, Z: 2}, true)
			defer runtime.KeepAlive(mGo)
			hRef := reference.Cube(2, 2, 2, true)
			defer reference.DeleteManifold(hRef)

			trimGo := mGo.TrimByPlane(tc.normal, tc.offset)
			defer runtime.KeepAlive(trimGo)
			trimRef := reference.TrimByPlane(hRef, tc.normal.X, tc.normal.Y, tc.normal.Z, tc.offset)
			defer reference.DeleteManifold(trimRef)

			if !floatClose(trimGo.Volume(), reference.Volume(trimRef), 1e-9, 1e-9) {
				t.Errorf("TrimByPlane Volume: go=%v ref=%v",
					trimGo.Volume(), reference.Volume(trimRef))
			}
			assertSameBoundingBox(t, trimGo.h, trimRef, 1e-10)
		})
	}
}

// TestWarp_TranslatesViaCallback applies a +1,+2,+3 translation to every
// vertex via the Warp callback and verifies the bounding box shifts by
// the same amount (and the volume is preserved).
func TestWarp_TranslatesViaCallback(t *testing.T) {
	m := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
	defer runtime.KeepAlive(m)
	warped := m.Warp(func(v *Vec3) {
		v.X += 1
		v.Y += 2
		v.Z += 3
	})
	defer runtime.KeepAlive(warped)

	bb := warped.BoundingBox()
	wantMin := Vec3{X: 0.5, Y: 1.5, Z: 2.5}
	wantMax := Vec3{X: 1.5, Y: 2.5, Z: 3.5}
	const eps = 1e-12
	if math.Abs(bb.Min.X-wantMin.X) > eps || math.Abs(bb.Min.Y-wantMin.Y) > eps || math.Abs(bb.Min.Z-wantMin.Z) > eps {
		t.Errorf("Warp Min: got %+v, want %+v", bb.Min, wantMin)
	}
	if math.Abs(bb.Max.X-wantMax.X) > eps || math.Abs(bb.Max.Y-wantMax.Y) > eps || math.Abs(bb.Max.Z-wantMax.Z) > eps {
		t.Errorf("Warp Max: got %+v, want %+v", bb.Max, wantMax)
	}
	if !floatClose(warped.Volume(), 1.0, 1e-12, 1e-12) {
		t.Errorf("Warp Volume: got %v, want 1", warped.Volume())
	}
}

// TestWarpBatch_TranslatesViaCallback exercises the batch version
// of Warp by translating every vertex with one call instead of N.
func TestWarpBatch_TranslatesViaCallback(t *testing.T) {
	m := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
	defer runtime.KeepAlive(m)
	called := false
	warped := m.WarpBatch(func(verts []Vec3) {
		called = true
		for i := range verts {
			verts[i].X += 1
			verts[i].Y += 2
			verts[i].Z += 3
		}
	})
	defer runtime.KeepAlive(warped)

	if !called {
		t.Error("WarpBatch callback was not invoked")
	}
	bb := warped.BoundingBox()
	const eps = 1e-12
	if math.Abs(bb.Min.X-0.5) > eps || math.Abs(bb.Max.X-1.5) > eps {
		t.Errorf("WarpBatch BoundingBox X: got [%v, %v], want [0.5, 1.5]", bb.Min.X, bb.Max.X)
	}
	if !floatClose(warped.Volume(), 1.0, 1e-12, 1e-12) {
		t.Errorf("WarpBatch Volume: got %v, want 1", warped.Volume())
	}
}

// TestRevolve_Differential exercises the inline Revolve port against
// the C++ reference. A unit square (axis-offset rectangle) revolved
// around Y produces a hollow torus-ish shape; the resulting Volume
// and bounding box must match the reference.
func TestRevolve_Differential(t *testing.T) {
	// A square offset from the Y axis: (1,0)-(2,0)-(2,1)-(1,1).
	square := SimplePolygon{
		{X: 1, Y: 0},
		{X: 2, Y: 0},
		{X: 2, Y: 1},
		{X: 1, Y: 1},
	}
	polys := Polygons{square}

	cases := []struct {
		name     string
		segments int
		degrees  float64
	}{
		{"full 32-seg", 32, 360},
		{"half 16-seg", 16, 180},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			mGo := Revolve(polys, tc.segments, tc.degrees)
			defer runtime.KeepAlive(mGo)
			mRef := reference.Revolve(polys, tc.segments, tc.degrees)
			defer reference.DeleteManifold(mRef)

			if mGo.Status() != NoError {
				t.Fatalf("Go Revolve Status: %v", mGo.Status())
			}
			if !floatClose(mGo.Volume(), reference.Volume(mRef), 1e-8, 1e-9) {
				t.Errorf("Volume: go=%v ref=%v", mGo.Volume(), reference.Volume(mRef))
			}
			if mGo.NumVert() != reference.NumVert(mRef) {
				t.Errorf("NumVert: go=%d ref=%d", mGo.NumVert(), reference.NumVert(mRef))
			}
			assertSameBoundingBox(t, mGo.h, mRef, 1e-9)
		})
	}
}

// TestRayCast_Cube fires a ray through the center of a unit cube and
// confirms it hits two faces — one on entry, one on exit — and the
// distances are between 0 and 1 (parametric along the segment).
func TestRayCast_Cube(t *testing.T) {
	m := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
	defer runtime.KeepAlive(m)
	hits := m.RayCast(Vec3{X: -2}, Vec3{X: 2})
	if len(hits) != 2 {
		t.Fatalf("RayCast through cube: got %d hits, want 2", len(hits))
	}
	for i, h := range hits {
		if h.Distance < 0 || h.Distance > 1 {
			t.Errorf("hit %d distance %v outside [0,1]", i, h.Distance)
		}
		// Position X should be -0.5 (entry) or 0.5 (exit).
		if math.Abs(h.Position.X) > 0.501 {
			t.Errorf("hit %d position X=%v outside cube face", i, h.Position.X)
		}
	}
}

// TestDecompose_TwoCubes verifies the top-down port of Manifold::Decompose:
// the union of two non-touching cubes decomposes into two components,
// and each component's volume matches the cube's expected volume.
func TestDecompose_TwoCubes(t *testing.T) {
	a := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
	b := Cube(Vec3{X: 1, Y: 1, Z: 1}, true).Translate(Vec3{X: 5})
	defer runtime.KeepAlive(a)
	defer runtime.KeepAlive(b)
	combined := a.Union(b)
	defer runtime.KeepAlive(combined)

	parts := combined.Decompose()
	if len(parts) != 2 {
		t.Fatalf("Decompose: got %d parts, want 2", len(parts))
	}
	for i, p := range parts {
		defer runtime.KeepAlive(p)
		if !floatClose(p.Volume(), 1.0, 1e-9, 1e-9) {
			t.Errorf("part %d Volume: got %v, want 1", i, p.Volume())
		}
	}
}

// TestDecompose_Connected returns the input as a single-element slice.
func TestDecompose_Connected(t *testing.T) {
	m := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
	defer runtime.KeepAlive(m)
	parts := m.Decompose()
	if len(parts) != 1 {
		t.Fatalf("Decompose: got %d parts, want 1", len(parts))
	}
}

// TestGetMeshGL exercises the float32 mesh export.
func TestGetMeshGL(t *testing.T) {
	m := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
	defer runtime.KeepAlive(m)
	mesh := m.GetMeshGL(-1)
	if mesh.NumProp < 3 {
		t.Errorf("NumProp: got %d, want >= 3", mesh.NumProp)
	}
	if len(mesh.VertProperties)/mesh.NumProp != m.NumVert() {
		t.Errorf("VertProperties: %d / %d = %d, want NumVert %d",
			len(mesh.VertProperties), mesh.NumProp,
			len(mesh.VertProperties)/mesh.NumProp, m.NumVert())
	}
	if len(mesh.TriVerts)/3 != m.NumTri() {
		t.Errorf("TriVerts: %d / 3 = %d, want NumTri %d",
			len(mesh.TriVerts), len(mesh.TriVerts)/3, m.NumTri())
	}
}

// TestNewManifoldFromMeshGL64_Roundtrip verifies the Manifold(MeshGL64)
// constructor by round-tripping: export → reconstruct → check volume,
// vert count, tri count match the original.
func TestNewManifoldFromMeshGL64_Roundtrip(t *testing.T) {
	orig := Cube(Vec3{X: 2, Y: 3, Z: 5}, true)
	defer runtime.KeepAlive(orig)
	mesh := orig.GetMeshGL64(-1)

	rebuilt := NewManifoldFromMeshGL64(mesh)
	defer runtime.KeepAlive(rebuilt)

	if rebuilt.Status() != NoError {
		t.Fatalf("rebuilt Status: got %v, want NoError", rebuilt.Status())
	}
	if !floatClose(rebuilt.Volume(), orig.Volume(), 1e-9, 1e-9) {
		t.Errorf("Volume: rebuilt=%v orig=%v", rebuilt.Volume(), orig.Volume())
	}
	if rebuilt.NumVert() != orig.NumVert() {
		t.Errorf("NumVert: rebuilt=%d orig=%d", rebuilt.NumVert(), orig.NumVert())
	}
	if rebuilt.NumTri() != orig.NumTri() {
		t.Errorf("NumTri: rebuilt=%d orig=%d", rebuilt.NumTri(), orig.NumTri())
	}
}

// TestNewManifoldFromMeshGL_Roundtrip exercises the float32 constructor.
func TestNewManifoldFromMeshGL_Roundtrip(t *testing.T) {
	orig := Cube(Vec3{X: 2, Y: 3, Z: 5}, true)
	defer runtime.KeepAlive(orig)
	mesh := orig.GetMeshGL(-1)
	rebuilt := NewManifoldFromMeshGL(mesh)
	defer runtime.KeepAlive(rebuilt)
	if rebuilt.Status() != NoError {
		t.Fatalf("rebuilt Status: %v", rebuilt.Status())
	}
	if !floatClose(rebuilt.Volume(), orig.Volume(), 1e-5, 1e-5) {
		t.Errorf("Volume: rebuilt=%v orig=%v", rebuilt.Volume(), orig.Volume())
	}
}

// TestGetMeshGL64 exercises the top-down port of Manifold::GetMeshGL64.
// A unit cube must export with non-empty VertProperties/TriVerts whose
// derived NumVert/NumTri match the source Manifold's accessors.
func TestGetMeshGL64(t *testing.T) {
	m := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
	defer runtime.KeepAlive(m)
	mesh := m.GetMeshGL64(-1)

	if mesh.NumProp < 3 {
		t.Errorf("NumProp: got %d, want >= 3", mesh.NumProp)
	}
	if len(mesh.VertProperties)/mesh.NumProp != m.NumVert() {
		t.Errorf("VertProperties: %d / %d = %d, want NumVert %d",
			len(mesh.VertProperties), mesh.NumProp,
			len(mesh.VertProperties)/mesh.NumProp, m.NumVert())
	}
	if len(mesh.TriVerts)/3 != m.NumTri() {
		t.Errorf("TriVerts: %d / 3 = %d, want NumTri %d",
			len(mesh.TriVerts), len(mesh.TriVerts)/3, m.NumTri())
	}

	// The first 3 properties of each vert are XYZ; they must fall inside
	// the unit cube's [-0.5, 0.5] extent.
	for v := 0; v < len(mesh.VertProperties)/mesh.NumProp; v++ {
		off := v * mesh.NumProp
		for axis := 0; axis < 3; axis++ {
			coord := mesh.VertProperties[off+axis]
			if coord < -0.501 || coord > 0.501 {
				t.Errorf("vert %d axis %d coord %v outside [-0.5, 0.5]",
					v, axis, coord)
			}
		}
	}
}

// TestSlice_PureGo exercises the top-down port of Manifold::Slice.
// The Z=0 slice of a centered 2x2x2 cube must produce at least one
// closed contour whose verts all lie in the cube's XY extent.
func TestSlice_PureGo(t *testing.T) {
	m := Cube(Vec3{X: 2, Y: 2, Z: 2}, true)
	defer runtime.KeepAlive(m)
	polys := m.Slice(0)
	if len(polys) == 0 {
		t.Fatalf("Slice cube@z=0: got 0 polys")
	}
	for i, p := range polys {
		if len(p) < 3 {
			t.Errorf("Slice poly %d has %d verts (must be a polygon)", i, len(p))
		}
		for j, v := range p {
			if v.X < -1.001 || v.X > 1.001 || v.Y < -1.001 || v.Y > 1.001 {
				t.Errorf("Slice poly %d vert %d outside XY [-1,1]: %+v",
					i, j, v)
			}
		}
	}
}

// TestProject_PureGo exercises the top-down port of Manifold::Project.
// Projection of a centered cube onto XY yields its XY footprint
// (possibly multiple coincident squares from front + back).
func TestProject_PureGo(t *testing.T) {
	m := Cube(Vec3{X: 2, Y: 2, Z: 2}, true)
	defer runtime.KeepAlive(m)
	polys := m.Project()
	if len(polys) == 0 {
		t.Fatalf("Project cube: got 0 polys")
	}
	// Each polygon's vertices must lie in the cube's XY extent.
	for i, p := range polys {
		for j, v := range p {
			if v.X < -1.001 || v.X > 1.001 || v.Y < -1.001 || v.Y > 1.001 {
				t.Errorf("Project poly %d vert %d outside XY [-1,1]: %+v",
					i, j, v)
			}
		}
	}
}

// TestCalculateCurvature_Differential exercises Manifold::CalculateCurvature
// against the reference. We use a sphere where curvature is meaningful
// (a flat-faced cube would have zero curvature everywhere).
func TestCalculateCurvature_Differential(t *testing.T) {
	mGo := Sphere(1.0, 32)
	defer runtime.KeepAlive(mGo)
	hRef := reference.Sphere(1.0, 32)
	defer reference.DeleteManifold(hRef)

	cGo := mGo.CalculateCurvature(0, 1)
	defer runtime.KeepAlive(cGo)
	cRef := reference.CalculateCurvature(hRef, 0, 1)
	defer reference.DeleteManifold(cRef)

	if got, want := cGo.NumProp(), reference.NumProp(cRef); got != want {
		t.Errorf("NumProp: go=%d ref=%d", got, want)
	}
	if !floatClose(cGo.Volume(), reference.Volume(cRef), 1e-9, 1e-9) {
		t.Errorf("Volume: go=%v ref=%v",
			cGo.Volume(), reference.Volume(cRef))
	}
}

// TestCalculateNormals_Differential exercises Manifold::CalculateNormals
// against the reference. After CalculateNormals(0, ...), NumProp must
// be at least 3 (xyz normals) and the resulting Volume must match.
func TestCalculateNormals_Differential(t *testing.T) {
	mGo := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
	defer runtime.KeepAlive(mGo)
	hRef := reference.Cube(1, 1, 1, true)
	defer reference.DeleteManifold(hRef)

	nGo := mGo.CalculateNormals(0, 60)
	defer runtime.KeepAlive(nGo)
	nRef := reference.CalculateNormals(hRef, 0, 60)
	defer reference.DeleteManifold(nRef)

	if got, want := nGo.NumProp(), reference.NumProp(nRef); got != want {
		t.Errorf("NumProp: go=%d ref=%d", got, want)
	}
	if !floatClose(nGo.Volume(), reference.Volume(nRef), 1e-9, 1e-9) {
		t.Errorf("Volume: go=%v ref=%v",
			nGo.Volume(), reference.Volume(nRef))
	}
}

// TestSimplify_Differential exercises Manifold::Simplify against the
// C++ reference at several tolerances.
func TestSimplify_Differential(t *testing.T) {
	cases := []struct {
		name string
		tol  float64
	}{
		{"tol=0 (use existing)", 0},
		{"tol=0.01", 0.01},
		{"tol=0.1", 0.1},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			mGo := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
			defer runtime.KeepAlive(mGo)
			hRef := reference.Cube(1, 1, 1, true)
			defer reference.DeleteManifold(hRef)

			sGo := mGo.Simplify(tc.tol)
			defer runtime.KeepAlive(sGo)
			sRef := reference.Simplify(hRef, tc.tol)
			defer reference.DeleteManifold(sRef)

			if !floatClose(sGo.Volume(), reference.Volume(sRef), 1e-9, 1e-9) {
				t.Errorf("Volume: go=%v ref=%v",
					sGo.Volume(), reference.Volume(sRef))
			}
			if got, want := sGo.NumVert(), reference.NumVert(sRef); got != want {
				t.Errorf("NumVert: go=%d ref=%d", got, want)
			}
		})
	}
}

// TestReserveIDs verifies our port returns sequential IDs that match the
// reference's counter (modulo cross-test counter state — we only check
// that successive IDs differ by the expected amount).
func TestReserveIDs(t *testing.T) {
	a := ReserveIDs(1)
	b := ReserveIDs(5)
	c := ReserveIDs(1)
	if b != a+1 {
		t.Errorf("ReserveIDs sequence: a=%d, next should be %d, got %d", a, a+1, b)
	}
	if c != b+5 {
		t.Errorf("ReserveIDs(5) should advance by 5: b=%d c=%d", b, c)
	}
}

// TestSmoothOut_SmokeRun calls SmoothOut on a tetrahedron and confirms
// it stays non-empty and statusless. SmoothOut derives its tangents from
// the triangle geometry, so it works on any closed Manifold (unlike
// SmoothByNormals, which expects pre-existing per-vertex normals at the
// requested property slot — not present in primitive shapes).
//
// After a subsequent Refine the tangents must produce additional verts.
func TestSmoothOut_SmokeRun(t *testing.T) {
	m := Tetrahedron()
	defer runtime.KeepAlive(m)

	smoothOut := m.SmoothOut(60, 0)
	defer runtime.KeepAlive(smoothOut)
	if smoothOut.IsEmpty() {
		t.Error("SmoothOut returned empty")
	}
	if smoothOut.Status() != NoError {
		t.Errorf("SmoothOut Status: %v", smoothOut.Status())
	}

	refined := smoothOut.Refine(4)
	defer runtime.KeepAlive(refined)
	if refined.NumVert() <= smoothOut.NumVert() {
		t.Errorf("Refine after SmoothOut should add verts: before=%d after=%d",
			smoothOut.NumVert(), refined.NumVert())
	}
}

// TestSetTolerance_Differential exercises the top-down port of
// SetTolerance against the C++ reference. Both the "increase tolerance"
// branch (which runs SetNormalsAndCoplanar/SimplifyTopology/SortGeometry)
// and the "reduce tolerance" branch (max(epsilon, tol)) are covered.
func TestSetTolerance_Differential(t *testing.T) {
	cases := []struct {
		name string
		tol  float64
	}{
		{"increase to 0.01", 0.01},
		{"increase to 0.1", 0.1},
		{"reduce below epsilon", 1e-20}, // clamps up to epsilon
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			mGo := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
			defer runtime.KeepAlive(mGo)
			hRef := reference.Cube(1, 1, 1, true)
			defer reference.DeleteManifold(hRef)

			setGo := mGo.SetTolerance(tc.tol)
			defer runtime.KeepAlive(setGo)
			setRef := reference.SetTolerance(hRef, tc.tol)
			defer reference.DeleteManifold(setRef)

			if !floatClose(setGo.GetTolerance(), reference.GetTolerance(setRef), 1e-15, 1e-15) {
				t.Errorf("GetTolerance: go=%v ref=%v",
					setGo.GetTolerance(), reference.GetTolerance(setRef))
			}
			if !floatClose(setGo.Volume(), reference.Volume(setRef), 1e-9, 1e-9) {
				t.Errorf("Volume: go=%v ref=%v",
					setGo.Volume(), reference.Volume(setRef))
			}
			if setGo.NumVert() != reference.NumVert(setRef) {
				t.Errorf("NumVert: go=%d ref=%d",
					setGo.NumVert(), reference.NumVert(setRef))
			}
		})
	}
}

// TestMoreAccessors_Tetrahedron_Differential covers NumProp, NumPropVert,
// Genus, GetTolerance, OriginalID.
func TestMoreAccessors_Tetrahedron_Differential(t *testing.T) {
	hOrig := reference.Tetrahedron()
	defer reference.DeleteManifold(hOrig)
	m := &Manifold{h: hOrig}

	if got, want := m.NumProp(), reference.NumProp(hOrig); got != want {
		t.Errorf("NumProp: go=%d ref=%d", got, want)
	}
	if got, want := m.NumPropVert(), reference.NumPropVert(hOrig); got != want {
		t.Errorf("NumPropVert: go=%d ref=%d", got, want)
	}
	if got, want := m.Genus(), reference.Genus(hOrig); got != want {
		t.Errorf("Genus: go=%d ref=%d", got, want)
	}
	if got, want := m.GetTolerance(), reference.GetTolerance(hOrig); !floatClose(got, want, 1e-15, 1e-15) {
		t.Errorf("GetTolerance: go=%v ref=%v", got, want)
	}
	if got, want := m.OriginalID(), reference.OriginalID(hOrig); got != want {
		t.Errorf("OriginalID: go=%d ref=%d", got, want)
	}
}

// TestSphereCylinder_GoFactory_Differential covers the Sphere and Cylinder
// factories.
func TestSphereCylinder_GoFactory_Differential(t *testing.T) {
	t.Run("sphere r=1", func(t *testing.T) {
		mGo := Sphere(1.0, 16)
		defer runtime.KeepAlive(mGo)
		hRef := reference.Sphere(1.0, 16)
		defer reference.DeleteManifold(hRef)
		assertSameBoundingBox(t, mGo.h, hRef, 1e-12)
		if got, want := mGo.NumVert(), reference.NumVert(hRef); got != want {
			t.Errorf("NumVert: go=%d ref=%d", got, want)
		}
	})
	t.Run("cylinder", func(t *testing.T) {
		mGo := Cylinder(5, 1, 1, 12, false)
		defer runtime.KeepAlive(mGo)
		hRef := reference.Cylinder(5, 1, 1, 12, false)
		defer reference.DeleteManifold(hRef)
		assertSameBoundingBox(t, mGo.h, hRef, 1e-12)
	})
	t.Run("frustum centered", func(t *testing.T) {
		mGo := Cylinder(2, 1, 0.5, 8, true)
		defer runtime.KeepAlive(mGo)
		hRef := reference.Cylinder(2, 1, 0.5, 8, true)
		defer reference.DeleteManifold(hRef)
		assertSameBoundingBox(t, mGo.h, hRef, 1e-12)
	})
}

// TestBooleanOps_Differential covers Union, Difference, Intersection on
// two overlapping cubes.
func TestBooleanOps_Differential(t *testing.T) {
	aGo := Cube(Vec3{X: 2, Y: 2, Z: 2}, true)
	bGo := Cube(Vec3{X: 2, Y: 2, Z: 2}, true).Translate(Vec3{X: 1, Y: 1, Z: 1})
	defer runtime.KeepAlive(aGo)
	defer runtime.KeepAlive(bGo)

	aRef := reference.Cube(2, 2, 2, true)
	defer reference.DeleteManifold(aRef)
	bRefBase := reference.Cube(2, 2, 2, true)
	defer reference.DeleteManifold(bRefBase)
	bRef := reference.Translate(bRefBase, Vec3{X: 1, Y: 1, Z: 1})
	defer reference.DeleteManifold(bRef)

	t.Run("union", func(t *testing.T) {
		uGo := aGo.Union(bGo)
		defer runtime.KeepAlive(uGo)
		uRef := reference.Union(aRef, bRef)
		defer reference.DeleteManifold(uRef)
		assertSameBoundingBox(t, uGo.h, uRef, 1e-10)
		if !floatClose(uGo.Volume(), reference.Volume(uRef), 1e-9, 1e-9) {
			t.Errorf("union volume: go=%v ref=%v", uGo.Volume(), reference.Volume(uRef))
		}
	})
	t.Run("difference", func(t *testing.T) {
		dGo := aGo.Difference(bGo)
		defer runtime.KeepAlive(dGo)
		dRef := reference.Difference(aRef, bRef)
		defer reference.DeleteManifold(dRef)
		if !floatClose(dGo.Volume(), reference.Volume(dRef), 1e-9, 1e-9) {
			t.Errorf("difference volume: go=%v ref=%v", dGo.Volume(), reference.Volume(dRef))
		}
	})
	t.Run("intersection", func(t *testing.T) {
		iGo := aGo.Intersection(bGo)
		defer runtime.KeepAlive(iGo)
		iRef := reference.Intersection(aRef, bRef)
		defer reference.DeleteManifold(iRef)
		assertSameBoundingBox(t, iGo.h, iRef, 1e-10)
		if !floatClose(iGo.Volume(), reference.Volume(iRef), 1e-9, 1e-9) {
			t.Errorf("intersection volume: go=%v ref=%v", iGo.Volume(), reference.Volume(iRef))
		}
	})
}

// TestStatus_Tetrahedron_Differential verifies Status returns NoError for
// a well-formed Manifold.
func TestStatus_Tetrahedron_Differential(t *testing.T) {
	m := Tetrahedron()
	defer runtime.KeepAlive(m)
	if m.Status() != NoError {
		t.Errorf("Tetrahedron Status: got %v, want NoError", m.Status())
	}
	if int(m.Status()) != reference.Status(m.h) {
		t.Errorf("Status: go=%d ref=%d", m.Status(), reference.Status(m.h))
	}
}

// TestHullPts_Differential exercises the top-down port of the static
// Manifold::Hull(pts) against the C++ reference.
func TestHullPts_Differential(t *testing.T) {
	pts := []Vec3{
		{X: -1, Y: -1, Z: 1},
		{X: -1, Y: 1, Z: -1},
		{X: 1, Y: -1, Z: -1},
		{X: 1, Y: 1, Z: 1},
		{X: 0, Y: 0, Z: 0},
	}
	mGo := HullPts(pts)
	defer runtime.KeepAlive(mGo)
	mRef := reference.HullPts(pts)
	defer reference.DeleteManifold(mRef)

	if !floatClose(mGo.Volume(), reference.Volume(mRef), 1e-12, 1e-12) {
		t.Errorf("HullPts Volume: go=%v ref=%v",
			mGo.Volume(), reference.Volume(mRef))
	}
	assertSameBoundingBox(t, mGo.h, mRef, 1e-12)
}

// TestBatchBoolean_Differential exercises Compose/BatchBoolean: union of
// three cubes laid out diagonally should match the equivalent built by
// pairwise Boolean.
func TestBatchBoolean_Compose_Differential(t *testing.T) {
	a := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
	b := Cube(Vec3{X: 1, Y: 1, Z: 1}, true).Translate(Vec3{X: 0.5})
	c := Cube(Vec3{X: 1, Y: 1, Z: 1}, true).Translate(Vec3{Y: 0.5})
	defer runtime.KeepAlive(a)
	defer runtime.KeepAlive(b)
	defer runtime.KeepAlive(c)

	batched := Compose([]*Manifold{a, b, c})
	defer runtime.KeepAlive(batched)
	pairwise := a.Union(b).Union(c)
	defer runtime.KeepAlive(pairwise)

	if !floatClose(batched.Volume(), pairwise.Volume(), 1e-9, 1e-9) {
		t.Errorf("Compose vs pairwise Volume: batch=%v pairwise=%v",
			batched.Volume(), pairwise.Volume())
	}
}

// TestBatchHull_Differential exercises the static Manifold::Hull(vector<Manifold>):
// the hull of two cubes equals the hull of their combined vertex sets.
func TestBatchHull_Differential(t *testing.T) {
	a := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
	b := Cube(Vec3{X: 1, Y: 1, Z: 1}, true).Translate(Vec3{X: 2})
	defer runtime.KeepAlive(a)
	defer runtime.KeepAlive(b)
	bh := BatchHull([]*Manifold{a, b})
	defer runtime.KeepAlive(bh)

	// Reference: collect vert positions of both, run HullPts on them.
	var allPts []Vec3
	for _, m := range []*Manifold{a, b} {
		impl := bridge.GetImpl(m.h)
		allPts = append(allPts, impl.Verts()...)
		impl.Delete()
	}
	hRef := reference.HullPts(allPts)
	defer reference.DeleteManifold(hRef)

	if !floatClose(bh.Volume(), reference.Volume(hRef), 1e-9, 1e-9) {
		t.Errorf("BatchHull Volume: go=%v ref=%v",
			bh.Volume(), reference.Volume(hRef))
	}
	assertSameBoundingBox(t, bh.h, hRef, 1e-10)
}

// TestHull_Differential checks that Hull on a Manifold (already convex
// tetrahedron) produces equivalent volume/bbox to the C++ reference.
func TestHull_Differential(t *testing.T) {
	hOrig := reference.Tetrahedron()
	defer reference.DeleteManifold(hOrig)
	m := &Manifold{h: hOrig}

	hGo := m.Hull()
	defer runtime.KeepAlive(hGo)
	hRef := reference.Hull(hOrig)
	defer reference.DeleteManifold(hRef)

	assertSameBoundingBox(t, hGo.h, hRef, 1e-12)
	if !floatClose(hGo.Volume(), reference.Volume(hRef), 1e-12, 1e-12) {
		t.Errorf("Hull volume: go=%v ref=%v", hGo.Volume(), reference.Volume(hRef))
	}
}

// TestRefine_Differential checks the three refine flavors all change the
// vertex count consistently with the C++ reference.
// TestRefine_Smoothed_VsReference differential-tests the native Refine SMOOTHING
// path (Impl::Refine + the InterpTri cubic-Bezier surface interpolation, incl. the
// quaternion machinery) against the C++ bridge. The input must carry halfedge
// tangents (else InterpTri is skipped), so each case smooths first, then Refines
// the SAME smoothed manifold both ways. Topology is deterministic (exact vert/tri
// counts); positions go through acos/sin/cos (Go stdlib vs C++ musl), so volume/
// area match semantically (~1e-6), not bit-for-bit.
func TestRefine_Smoothed_VsReference(t *testing.T) {
	cases := []struct {
		name string
		mk   func() *Manifold
	}{
		{"smoothbynormals_sphere", func() *Manifold {
			return Sphere(1, 16).CalculateNormals(0, 0).SmoothByNormals(0)
		}},
		{"smoothout_sphere", func() *Manifold {
			return Sphere(1, 12).SmoothOut(60, 0)
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			sm := tc.mk()
			defer runtime.KeepAlive(sm)
			if sm.IsEmpty() || sm.Status() != NoError {
				t.Fatalf("smoothing produced empty/errored input: empty=%v status=%v", sm.IsEmpty(), sm.Status())
			}

			got := sm.Refine(3) // native: Impl::Refine + InterpTri
			defer runtime.KeepAlive(got)
			ref := reference.Refine(sm.h, 3) // C++ bridge on the identical smoothed input
			defer reference.DeleteManifold(ref)

			if g, w := got.NumVert(), reference.NumVert(ref); g != w {
				t.Errorf("NumVert: native=%d ref=%d", g, w)
			}
			if g, w := got.NumTri(), reference.NumTri(ref); g != w {
				t.Errorf("NumTri: native=%d ref=%d", g, w)
			}
			if !floatClose(got.Volume(), reference.Volume(ref), 1e-6, 1e-6) {
				t.Errorf("Volume: native=%v ref=%v", got.Volume(), reference.Volume(ref))
			}
			if !floatClose(got.SurfaceArea(), reference.SurfaceArea(ref), 1e-6, 1e-6) {
				t.Errorf("SurfaceArea: native=%v ref=%v", got.SurfaceArea(), reference.SurfaceArea(ref))
			}
		})
	}
}

func TestRefine_Differential(t *testing.T) {
	t.Run("refine(2)", func(t *testing.T) {
		hOrig := reference.Tetrahedron()
		defer reference.DeleteManifold(hOrig)
		m := &Manifold{h: hOrig}
		rGo := m.Refine(2)
		defer runtime.KeepAlive(rGo)
		rRef := reference.Refine(hOrig, 2)
		defer reference.DeleteManifold(rRef)
		if rGo.NumVert() != reference.NumVert(rRef) {
			t.Errorf("Refine(2) NumVert: go=%d ref=%d", rGo.NumVert(), reference.NumVert(rRef))
		}
	})
	t.Run("refine_to_length(0.5)", func(t *testing.T) {
		hOrig := reference.Tetrahedron()
		defer reference.DeleteManifold(hOrig)
		m := &Manifold{h: hOrig}
		rGo := m.RefineToLength(0.5)
		defer runtime.KeepAlive(rGo)
		rRef := reference.RefineToLength(hOrig, 0.5)
		defer reference.DeleteManifold(rRef)
		if rGo.NumVert() != reference.NumVert(rRef) {
			t.Errorf("RefineToLength NumVert: go=%d ref=%d", rGo.NumVert(), reference.NumVert(rRef))
		}
	})
}

// TestMinGap_Differential exercises the top-down port of MinGap against
// the C++ reference. Two disjoint cubes (overlap = 0) should report the
// edge-to-edge distance; overlapping cubes should report 0.
func TestMinGap_Differential(t *testing.T) {
	t.Run("disjoint", func(t *testing.T) {
		aGo := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
		bGo := Cube(Vec3{X: 1, Y: 1, Z: 1}, true).Translate(Vec3{X: 3})
		defer runtime.KeepAlive(aGo)
		defer runtime.KeepAlive(bGo)
		aRef := reference.Cube(1, 1, 1, true)
		defer reference.DeleteManifold(aRef)
		bRefBase := reference.Cube(1, 1, 1, true)
		defer reference.DeleteManifold(bRefBase)
		bRef := reference.Translate(bRefBase, Vec3{X: 3})
		defer reference.DeleteManifold(bRef)

		got := aGo.MinGap(bGo, 5)
		want := reference.MinGap(aRef, bRef, 5)
		if !floatClose(got, want, 1e-9, 1e-9) {
			t.Errorf("disjoint MinGap: go=%v ref=%v", got, want)
		}
	})
	t.Run("overlapping returns 0", func(t *testing.T) {
		aGo := Cube(Vec3{X: 2, Y: 2, Z: 2}, true)
		bGo := Cube(Vec3{X: 2, Y: 2, Z: 2}, true).Translate(Vec3{X: 0.5})
		defer runtime.KeepAlive(aGo)
		defer runtime.KeepAlive(bGo)
		if got := aGo.MinGap(bGo, 5); got != 0 {
			t.Errorf("overlapping MinGap: got %v, want 0", got)
		}
	})
}

// TestMinkowski_Differential covers MinkowskiSum on two cubes.
func TestMinkowski_Differential(t *testing.T) {
	aGo := Cube(Vec3{X: 1, Y: 1, Z: 1}, true)
	bGo := Cube(Vec3{X: 0.5, Y: 0.5, Z: 0.5}, true)
	defer runtime.KeepAlive(aGo)
	defer runtime.KeepAlive(bGo)
	aRef := reference.Cube(1, 1, 1, true)
	defer reference.DeleteManifold(aRef)
	bRef := reference.Cube(0.5, 0.5, 0.5, true)
	defer reference.DeleteManifold(bRef)

	sGo := aGo.MinkowskiSum(bGo)
	defer runtime.KeepAlive(sGo)
	sRef := reference.MinkowskiSum(aRef, bRef)
	defer reference.DeleteManifold(sRef)

	if !floatClose(sGo.Volume(), reference.Volume(sRef), 1e-9, 1e-9) {
		t.Errorf("MinkowskiSum volume: go=%v ref=%v", sGo.Volume(), reference.Volume(sRef))
	}
}

// TestMinkowski_Native_VsReference exercises the non-convex paths of the native
// Minkowski (impl_minkowski.go) against the C++ bridge on identical inputs. The
// convex×non-convex and non-convex×non-convex branches build per-feature hulls
// and union them via the native BatchBoolean; the inset case drives the
// difference (erosion) path. Non-convex operands are L-shapes (a cube unioned
// with an offset cube — one connected, non-convex manifold).
func TestMinkowski_Native_VsReference(t *testing.T) {
	smallCube := func() *Manifold { return Cube(Vec3{X: 0.4, Y: 0.4, Z: 0.4}, true) }
	lShape := func() *Manifold {
		return Cube(Vec3{X: 1, Y: 1, Z: 1}, true).
			Union(Cube(Vec3{X: 1, Y: 1, Z: 1}, true).Translate(Vec3{X: 0.8, Y: 0.8}))
	}
	cases := []struct {
		name  string
		a, b  func() *Manifold
		inset bool
	}{
		{"convex_nonconvex_sum", smallCube, lShape, false},   // swap -> a=L, b=cube, convex-RHS batch path
		{"nonconvex_nonconvex_sum", lShape, lShape, false},   // slow per-face-pair path
		{"inset_difference", lShape, smallCube, true},        // erosion / Subtract finalize
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			a := tc.a()
			b := tc.b()
			defer runtime.KeepAlive(a)
			defer runtime.KeepAlive(b)

			var got *Manifold
			var refVol, refArea float64
			var refGenus int
			if tc.inset {
				got = a.MinkowskiDifference(b)
				r := reference.MinkowskiDifference(a.h, b.h)
				refVol, refArea, refGenus = reference.Volume(r), reference.SurfaceArea(r), reference.Genus(r)
				reference.DeleteManifold(r)
			} else {
				got = a.MinkowskiSum(b)
				r := reference.MinkowskiSum(a.h, b.h)
				refVol, refArea, refGenus = reference.Volume(r), reference.SurfaceArea(r), reference.Genus(r)
				reference.DeleteManifold(r)
			}
			defer runtime.KeepAlive(got)

			if !floatClose(got.Volume(), refVol, 1e-7, 1e-7) {
				t.Errorf("volume: native=%v ref=%v", got.Volume(), refVol)
			}
			if !floatClose(got.SurfaceArea(), refArea, 1e-7, 1e-7) {
				t.Errorf("surfaceArea: native=%v ref=%v", got.SurfaceArea(), refArea)
			}
			if g := got.Genus(); g != refGenus {
				t.Errorf("genus: native=%d ref=%d", g, refGenus)
			}
		})
	}
}

// TestTransform_Tetrahedron_Differential exercises the public Transform
// method by applying a hand-built mat3x4 (here: scale 2x along X, rotate
// implicitly via off-diagonal entry, translate by (1,0,0)) and comparing
// to the C++ reference Transform.
func TestTransform_Tetrahedron_Differential(t *testing.T) {
	hOrig := reference.Tetrahedron()
	defer reference.DeleteManifold(hOrig)

	m := Mat3x4{
		{2, 0, 0},   // col 0
		{0, 1, 0},   // col 1
		{0, 0, 1},   // col 2
		{1, 0, 0.5}, // col 3 (translation)
	}
	mManifold := &Manifold{h: hOrig}
	mGo := mManifold.Transform(m)
	defer runtime.KeepAlive(mGo)

	hRef := reference.Transform(hOrig,
		m[0][0], m[0][1], m[0][2],
		m[1][0], m[1][1], m[1][2],
		m[2][0], m[2][1], m[2][2],
		m[3][0], m[3][1], m[3][2],
	)
	defer reference.DeleteManifold(hRef)

	assertSameBoundingBox(t, mGo.h, hRef, 1e-12)
	if !floatClose(mGo.Volume(), reference.Volume(hRef), 1e-12, 1e-12) {
		t.Errorf("transformed Volume: go=%v ref=%v", mGo.Volume(), reference.Volume(hRef))
	}
}

// TestTransform_NegativeDet_Differential exercises the determinant < 0
// branch of the drilled Transform: a transform that scales by -1 in one
// axis (and so should flip triangle orientation). Volume must still
// match the reference; if the flip is missing or done wrong, volume
// goes negative or shapes are inverted.
func TestTransform_NegativeDet_Differential(t *testing.T) {
	hOrig := reference.Tetrahedron()
	defer reference.DeleteManifold(hOrig)

	m := Mat3x4{
		{-1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
		{0, 0, 0},
	}
	mGo := (&Manifold{h: hOrig}).Transform(m)
	defer runtime.KeepAlive(mGo)

	hRef := reference.Transform(hOrig,
		m[0][0], m[0][1], m[0][2],
		m[1][0], m[1][1], m[1][2],
		m[2][0], m[2][1], m[2][2],
		m[3][0], m[3][1], m[3][2],
	)
	defer reference.DeleteManifold(hRef)

	assertSameBoundingBox(t, mGo.h, hRef, 1e-12)
	if !floatClose(mGo.Volume(), reference.Volume(hRef), 1e-12, 1e-12) {
		t.Errorf("neg-det Volume: go=%v ref=%v", mGo.Volume(), reference.Volume(hRef))
	}
}

// TestSplit_TwoCubes_Differential cuts a unit cube with an
// overlapping cube and asserts the Go (intersection, difference)
// volumes match the C++ reference.
func TestSplit_TwoCubes_Differential(t *testing.T) {
	mGoA := Cube(Vec3{X: 2, Y: 2, Z: 2}, true)
	mGoB := Cube(Vec3{X: 2, Y: 2, Z: 2}, true).Translate(Vec3{X: 1, Y: 1, Z: 1})
	defer runtime.KeepAlive(mGoA)
	defer runtime.KeepAlive(mGoB)

	gotInter, gotDiff := mGoA.Split(mGoB)
	defer runtime.KeepAlive(gotInter)
	defer runtime.KeepAlive(gotDiff)

	hRefA := reference.Cube(2, 2, 2, true)
	defer reference.DeleteManifold(hRefA)
	hRefB := reference.Cube(2, 2, 2, true)
	hRefB = reference.Translate(hRefB, Vec3{X: 1, Y: 1, Z: 1})
	defer reference.DeleteManifold(hRefB)

	wantInter, wantDiff := reference.Split(hRefA, hRefB)
	defer reference.DeleteManifold(wantInter)
	defer reference.DeleteManifold(wantDiff)

	if !floatClose(gotInter.Volume(), reference.Volume(wantInter), 1e-9, 1e-9) {
		t.Errorf("Split intersection volume: go=%v ref=%v",
			gotInter.Volume(), reference.Volume(wantInter))
	}
	if !floatClose(gotDiff.Volume(), reference.Volume(wantDiff), 1e-9, 1e-9) {
		t.Errorf("Split difference volume: go=%v ref=%v",
			gotDiff.Volume(), reference.Volume(wantDiff))
	}
	assertSameBoundingBox(t, gotInter.h, wantInter, 1e-9)
	assertSameBoundingBox(t, gotDiff.h, wantDiff, 1e-9)
}

// TestSplitByPlane_Cube_Differential bisects a unit cube with a
// plane and asserts both halves match the C++ reference.
func TestSplitByPlane_Cube_Differential(t *testing.T) {
	mGo := Cube(Vec3{X: 2, Y: 2, Z: 2}, true)
	defer runtime.KeepAlive(mGo)

	normal := Vec3{X: 1, Y: 0, Z: 0}
	gotPos, gotNeg := mGo.SplitByPlane(normal, 0)
	defer runtime.KeepAlive(gotPos)
	defer runtime.KeepAlive(gotNeg)

	hRef := reference.Cube(2, 2, 2, true)
	defer reference.DeleteManifold(hRef)
	wantPos, wantNeg := reference.SplitByPlane(hRef, normal.X, normal.Y, normal.Z, 0)
	defer reference.DeleteManifold(wantPos)
	defer reference.DeleteManifold(wantNeg)

	if !floatClose(gotPos.Volume(), reference.Volume(wantPos), 1e-9, 1e-9) {
		t.Errorf("SplitByPlane +side volume: go=%v ref=%v",
			gotPos.Volume(), reference.Volume(wantPos))
	}
	if !floatClose(gotNeg.Volume(), reference.Volume(wantNeg), 1e-9, 1e-9) {
		t.Errorf("SplitByPlane -side volume: go=%v ref=%v",
			gotNeg.Volume(), reference.Volume(wantNeg))
	}
}

// TestMatchesTriNormals_Tetrahedron exercises the test-hook accessor.
// A tetrahedron from the standard factory must pass the consistency
// check.
func TestMatchesTriNormals_Tetrahedron(t *testing.T) {
	hOrig := reference.Tetrahedron()
	defer reference.DeleteManifold(hOrig)
	m := &Manifold{h: hOrig}
	if !m.MatchesTriNormals() {
		t.Error("Tetrahedron should have matching tri normals")
	}
}

// TestNumDegenerateTris_Tetrahedron checks that a clean tetrahedron
// has zero degenerate triangles. NB: the C++ early-return for an Impl
// with no cached face normals is "true" (which a tetrahedron from a
// boolean op can hit) — but Tetrahedron() returns an Impl with normals
// cached, so we get the loop's count: 0.
func TestNumDegenerateTris_Tetrahedron(t *testing.T) {
	hOrig := reference.Tetrahedron()
	defer reference.DeleteManifold(hOrig)
	m := &Manifold{h: hOrig}
	if got := m.NumDegenerateTris(); got != 0 {
		t.Errorf("Tetrahedron NumDegenerateTris: got %d, want 0", got)
	}
}

// TestMatchesTriNormals_DrilledVsCpp_Differential asserts the
// drilled-Go implementation of Impl::MatchesTriNormals agrees with
// the C++ Impl::MatchesTriNormals (still accessible through the
// bridge) for a range of inputs:
//   - a clean factory tetrahedron
//   - the union of two overlapping cubes (boolean result; halfedge_
//     and faceNormal_ may have different shapes than a factory mesh)
//   - a cube intersected with a sphere (mixed topology + many tris)
func TestMatchesTriNormals_DrilledVsCpp_Differential(t *testing.T) {
	cases := []struct {
		name string
		make func() *Manifold
	}{
		{"tetrahedron", func() *Manifold {
			return &Manifold{h: reference.Tetrahedron()}
		}},
		{"cube union", func() *Manifold {
			a := Cube(Vec3{X: 2, Y: 2, Z: 2}, true)
			b := Cube(Vec3{X: 2, Y: 2, Z: 2}, true).Translate(Vec3{X: 1, Y: 1, Z: 1})
			return a.Union(b)
		}},
		{"cube ^ sphere", func() *Manifold {
			c := Cube(Vec3{X: 2, Y: 2, Z: 2}, true)
			s := Sphere(1.2, 24)
			return c.Intersection(s)
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			m := tc.make()
			defer runtime.KeepAlive(m)
			impl := bridge.GetImpl(m.h)
			defer impl.Delete()
			gotGo := m.MatchesTriNormals() // drilled Go path
			gotCpp := impl.MatchesTriNormals()
			if gotGo != gotCpp {
				t.Errorf("MatchesTriNormals: drilled-Go=%v cpp=%v", gotGo, gotCpp)
			}
		})
	}
}

// TestNumDegenerateTris_DrilledVsCpp_Differential is the analog for
// NumDegenerateTris.
func TestNumDegenerateTris_DrilledVsCpp_Differential(t *testing.T) {
	cases := []struct {
		name string
		make func() *Manifold
	}{
		{"tetrahedron", func() *Manifold {
			return &Manifold{h: reference.Tetrahedron()}
		}},
		{"cube union", func() *Manifold {
			a := Cube(Vec3{X: 2, Y: 2, Z: 2}, true)
			b := Cube(Vec3{X: 2, Y: 2, Z: 2}, true).Translate(Vec3{X: 1, Y: 1, Z: 1})
			return a.Union(b)
		}},
		{"cube ^ sphere", func() *Manifold {
			c := Cube(Vec3{X: 2, Y: 2, Z: 2}, true)
			s := Sphere(1.2, 24)
			return c.Intersection(s)
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			m := tc.make()
			defer runtime.KeepAlive(m)
			impl := bridge.GetImpl(m.h)
			defer impl.Delete()
			gotGo := m.NumDegenerateTris()
			gotCpp := impl.NumDegenerateTris()
			if gotGo != gotCpp {
				t.Errorf("NumDegenerateTris: drilled-Go=%d cpp=%d", gotGo, gotCpp)
			}
		})
	}
}

// TestGetEpsilon_Tetrahedron checks GetEpsilon returns a positive
// value for a well-formed manifold.
func TestGetEpsilon_Tetrahedron(t *testing.T) {
	hOrig := reference.Tetrahedron()
	defer reference.DeleteManifold(hOrig)
	m := &Manifold{h: hOrig}
	if got := m.GetEpsilon(); !(got > 0) {
		t.Errorf("Tetrahedron GetEpsilon: got %v, want > 0", got)
	}
}

// TestSmoothFromMeshGL64_SmokeRun feeds a tetrahedron mesh through the
// Smooth factory with no sharpened edges, then refines once. The
// resulting Manifold must have NoError status, non-empty geometry, and
// preserve volume/bounding-box characteristics (smoothing+refining
// without sharpened edges shrinks volume but should remain positive
// and inside the bounding-box bounds).
func TestSmoothFromMeshGL64_SmokeRun(t *testing.T) {
	// Build a unit tetrahedron MeshGL64 from the C++ reference, then
	// feed it back through Smooth.
	hRef := reference.Tetrahedron()
	defer reference.DeleteManifold(hRef)
	mRef := &Manifold{h: hRef}
	mesh := mRef.GetMeshGL64(-1)
	mesh.HalfedgeTangent = nil // Smooth refuses tangented input.

	smoothed := SmoothFromMeshGL64(mesh, nil)
	defer runtime.KeepAlive(smoothed)
	if smoothed.Status() != NoError {
		t.Fatalf("SmoothFromMeshGL64 status: %v", smoothed.Status())
	}
	refined := smoothed.Refine(2)
	defer runtime.KeepAlive(refined)
	if refined.IsEmpty() {
		t.Fatal("refined smooth tetrahedron is empty")
	}
	if v := refined.Volume(); !(v > 0) {
		t.Errorf("refined smooth tetrahedron Volume: %v, want > 0", v)
	}
}

// TestExecutionContext_FreshState verifies a fresh context starts
// uncancelled and at Progress=1.0 (no work scheduled).
func TestExecutionContext_FreshState(t *testing.T) {
	ctx := NewExecutionContext()
	defer ctx.Delete()
	if ctx.Cancelled() {
		t.Error("fresh context should not be Cancelled")
	}
	if got := ctx.Progress(); got != 1.0 {
		t.Errorf("fresh context Progress: got %v, want 1.0", got)
	}
}

// TestExecutionContext_StickyCancel verifies that Cancel toggles
// Cancelled() and is idempotent. Whether an in-flight evaluation
// actually short-circuits depends on the C++ side's per-op
// cancellation granularity (small ops may complete before any check);
// here we only assert the API contract on the context object itself.
func TestExecutionContext_StickyCancel(t *testing.T) {
	ctx := NewExecutionContext()
	defer ctx.Delete()
	if ctx.Cancelled() {
		t.Fatal("fresh ctx should not be Cancelled")
	}
	ctx.Cancel()
	if !ctx.Cancelled() {
		t.Error("post-Cancel: Cancelled should be true")
	}
	// Idempotent: a second Cancel is a no-op and leaves Cancelled set.
	ctx.Cancel()
	if !ctx.Cancelled() {
		t.Error("second Cancel: Cancelled should remain true")
	}
}

// TestWithContext_NoCancel_ProducesSameResult exercises the happy
// path: WithContext on a fresh, uncancelled context must not alter
// the geometric result.
func TestWithContext_NoCancel_ProducesSameResult(t *testing.T) {
	ctx := NewExecutionContext()
	defer ctx.Delete()

	mPlain := Cube(Vec3{X: 2, Y: 2, Z: 2}, true)
	defer runtime.KeepAlive(mPlain)
	mCtx := mPlain.WithContext(ctx)
	defer runtime.KeepAlive(mCtx)

	if !floatClose(mCtx.Volume(), mPlain.Volume(), 1e-15, 1e-15) {
		t.Errorf("Volume differs: plain=%v ctx=%v", mPlain.Volume(), mCtx.Volume())
	}
	assertSameBoundingBox(t, mCtx.h, mPlain.h, 1e-15)
}

// TestSetProperties_PerVertexPosition writes the vertex position into
// the three user property channels, then verifies via GetMeshGL64
// that each propVert's user-property slot matches its position slot.
// MeshGL64.NumProp = 3 (position) + Impl::numProp_, so user props live
// at offsets 3..6 within each propVert.
func TestSetProperties_PerVertexPosition(t *testing.T) {
	hRef := reference.Tetrahedron()
	defer reference.DeleteManifold(hRef)
	src := &Manifold{h: hRef}

	out := src.SetProperties(3, func(newProp []float64, pos Vec3, _ []float64) {
		newProp[0] = pos.X
		newProp[1] = pos.Y
		newProp[2] = pos.Z
	})
	defer runtime.KeepAlive(out)
	if out.Status() != NoError {
		t.Fatalf("SetProperties status: %v", out.Status())
	}
	if got := out.NumProp(); got != 3 {
		t.Fatalf("NumProp after SetProperties: got %d, want 3", got)
	}
	mesh := out.GetMeshGL64(-1)
	npv := out.NumPropVert()
	stride := mesh.NumProp // 3 (position) + 3 (user)
	if stride != 6 {
		t.Fatalf("expected mesh.NumProp=6 (3 pos + 3 user), got %d", stride)
	}
	if len(mesh.VertProperties) != stride*npv {
		t.Fatalf("VertProperties len: got %d, want %d", len(mesh.VertProperties), stride*npv)
	}
	for i := 0; i < npv; i++ {
		off := stride * i
		px, py, pz := mesh.VertProperties[off+0], mesh.VertProperties[off+1], mesh.VertProperties[off+2]
		ux, uy, uz := mesh.VertProperties[off+3], mesh.VertProperties[off+4], mesh.VertProperties[off+5]
		if px != ux || py != uy || pz != uz {
			t.Errorf("propVert %d: pos=(%v,%v,%v) user=(%v,%v,%v)", i, px, py, pz, ux, uy, uz)
		}
	}
}

// TestSetProperties_NilCallback_ZeroFills exercises the nullptr path:
// passing fn=nil should produce a Manifold whose user properties are
// all zero. Verified via GetMeshGL64 (positions at offsets 0..2 remain
// non-zero; user channels at offsets 3..numProp must be zero).
func TestSetProperties_NilCallback_ZeroFills(t *testing.T) {
	hRef := reference.Tetrahedron()
	defer reference.DeleteManifold(hRef)
	src := &Manifold{h: hRef}

	out := src.SetProperties(2, nil)
	defer runtime.KeepAlive(out)
	if out.Status() != NoError {
		t.Fatalf("SetProperties(nil) status: %v", out.Status())
	}
	if got := out.NumProp(); got != 2 {
		t.Fatalf("NumProp: got %d, want 2", got)
	}
	mesh := out.GetMeshGL64(-1)
	stride := mesh.NumProp // 3 + 2
	if stride != 5 {
		t.Fatalf("expected mesh.NumProp=5, got %d", stride)
	}
	npv := out.NumPropVert()
	for i := 0; i < npv; i++ {
		for j := 3; j < stride; j++ {
			if v := mesh.VertProperties[stride*i+j]; v != 0 {
				t.Errorf("propVert %d channel %d: got %v, want 0", i, j, v)
			}
		}
	}
}

// TestLevelSet_Sphere_SmokeRun runs LevelSet on the unit-sphere SDF
// and checks the result is closed, non-empty, and roughly the right
// size. canParallel=false so the Go callback runs serially.
func TestLevelSet_Sphere_SmokeRun(t *testing.T) {
	radius := 1.0
	sdf := func(p Vec3) float64 {
		// Negative inside the sphere, positive outside — the C++
		// convention is the same.
		return radius - math.Sqrt(p.X*p.X+p.Y*p.Y+p.Z*p.Z)
	}
	bounds := Box{
		Min: Vec3{X: -2, Y: -2, Z: -2},
		Max: Vec3{X: 2, Y: 2, Z: 2},
	}
	m := LevelSet(sdf, bounds, 0.2, 0, -1, false)
	defer runtime.KeepAlive(m)

	if m.Status() != NoError {
		t.Fatalf("LevelSet status: %v", m.Status())
	}
	if m.IsEmpty() {
		t.Fatal("LevelSet sphere came back empty")
	}
	// Sphere volume ≈ 4/3 π r^3 ≈ 4.19; with edgeLength=0.2 the discrete
	// mesh is a faceted approximation, so allow generous slack.
	got := m.Volume()
	want := (4.0 / 3.0) * math.Pi * radius * radius * radius
	if math.Abs(got-want) > 0.5 {
		t.Errorf("LevelSet sphere Volume: got %v, want ≈ %v", got, want)
	}
}

// TestLevelSet_Sphere_Parallel exercises canParallel=true. The SDF is
// re-entrant (only reads its arguments), so it's safe under TBB.
func TestLevelSet_Sphere_Parallel(t *testing.T) {
	sdf := func(p Vec3) float64 {
		return 1.0 - math.Sqrt(p.X*p.X+p.Y*p.Y+p.Z*p.Z)
	}
	bounds := Box{
		Min: Vec3{X: -1.5, Y: -1.5, Z: -1.5},
		Max: Vec3{X: 1.5, Y: 1.5, Z: 1.5},
	}
	m := LevelSet(sdf, bounds, 0.15, 0, -1, true)
	defer runtime.KeepAlive(m)
	if m.Status() != NoError {
		t.Fatalf("LevelSet parallel status: %v", m.Status())
	}
	if m.IsEmpty() {
		t.Fatal("LevelSet parallel sphere came back empty")
	}
	if v := m.Volume(); !(v > 3.0 && v < 5.0) {
		t.Errorf("LevelSet parallel sphere Volume: got %v, want roughly 4.19", v)
	}
}

// TestSmoothFromMeshGL_SmokeRun is the float32/uint32 analog of
// TestSmoothFromMeshGL64_SmokeRun.
func TestSmoothFromMeshGL_SmokeRun(t *testing.T) {
	hRef := reference.Tetrahedron()
	defer reference.DeleteManifold(hRef)
	mRef := &Manifold{h: hRef}
	mesh := mRef.GetMeshGL(-1)
	mesh.HalfedgeTangent = nil

	smoothed := SmoothFromMeshGL(mesh, nil)
	defer runtime.KeepAlive(smoothed)
	if smoothed.Status() != NoError {
		t.Fatalf("SmoothFromMeshGL status: %v", smoothed.Status())
	}
	refined := smoothed.Refine(2)
	defer runtime.KeepAlive(refined)
	if refined.IsEmpty() {
		t.Fatal("refined smooth tetrahedron is empty")
	}
	if v := refined.Volume(); !(v > 0) {
		t.Errorf("refined smooth tetrahedron Volume: %v, want > 0", v)
	}
}
