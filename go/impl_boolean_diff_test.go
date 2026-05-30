package manifold

import (
	"math"
	"runtime"
	"sort"
	"testing"
)

// TestNativeBoolean_Properties validates createProperties' numProp>0 path: the
// native Boolean on property-carrying meshes (vertex normals) must match the C++
// bridge oracle in NumProp, NumPropVert, geometry, AND the interpolated property
// values (compared as an order-insensitive multiset of the raw properties_).
func TestNativeBoolean_Properties(t *testing.T) {
	// Two normal-carrying shapes (CalculateNormals adds vertex normals as props).
	ma := Sphere(1.0, 24).CalculateNormals(3, 60)
	mb := Cube(Vec3{X: 1.5, Y: 1.5, Z: 1.5}, true).Translate(Vec3{X: 0.4, Y: 0.4, Z: 0.4}).CalculateNormals(3, 60)
	defer runtime.KeepAlive(ma)
	defer runtime.KeepAlive(mb)
	approx := func(a, b float64) bool { return math.Abs(a-b) <= 1e-8*(1+math.Abs(b)) }

	sortedProps := func(m *Manifold) []float64 {
		v := getImpl(m)
		defer v.Delete()
		p := append([]float64(nil), v.Properties()...)
		sort.Float64s(p)
		return p
	}

	for _, tc := range []struct {
		op   OpType
		name string
	}{{OpAdd, "Add"}, {OpSubtract, "Subtract"}, {OpIntersect, "Intersect"}} {
		bridgeRes := ma.Boolean(mb, tc.op)

		va := getImpl(ma)
		vb := getImpl(mb)
		nb := newNativeBoolean3(va, vb, tc.op)
		nativeRes := nb.Result(tc.op)
		va.Delete()
		vb.Delete()

		if got, want := nativeRes.NumProp(), bridgeRes.NumProp(); got != want {
			t.Errorf("%s: NumProp native=%d bridge=%d", tc.name, got, want)
		}
		if got, want := nativeRes.NumPropVert(), bridgeRes.NumPropVert(); got != want {
			t.Errorf("%s: NumPropVert native=%d bridge=%d", tc.name, got, want)
		}
		if got, want := nativeRes.Volume(), bridgeRes.Volume(); !approx(got, want) {
			t.Errorf("%s: Volume native=%.12g bridge=%.12g", tc.name, got, want)
		}
		// Interpolated property values as an order-insensitive multiset.
		np, bp := sortedProps(nativeRes), sortedProps(bridgeRes)
		if len(np) != len(bp) {
			t.Fatalf("%s: properties_ len native=%d bridge=%d", tc.name, len(np), len(bp))
		}
		for i := range np {
			if !approx(np[i], bp[i]) {
				t.Errorf("%s: sorted properties_[%d] native=%.10g bridge=%.10g", tc.name, i, np[i], bp[i])
				break
			}
		}
	}
}

// compareBooleanVsBridge runs all three ops on (ma, mb) through both the native
// Boolean (nativeBoolean3.Result) and the bridge Boolean (the C++ oracle) and
// asserts the geometry matches: Volume/SurfaceArea (relative tol), Genus, and the
// exact vert/tri counts.
func compareBooleanVsBridge(t *testing.T, name string, ma, mb *Manifold) {
	t.Helper()
	defer runtime.KeepAlive(ma)
	defer runtime.KeepAlive(mb)
	approx := func(a, b float64) bool { return math.Abs(a-b) <= 1e-8*(1+math.Abs(b)) }

	for _, tc := range []struct {
		op   OpType
		name string
	}{{OpAdd, "Add"}, {OpSubtract, "Subtract"}, {OpIntersect, "Intersect"}} {
		bridgeRes := ma.Boolean(mb, tc.op)

		va := getImpl(ma)
		vb := getImpl(mb)
		nb := newNativeBoolean3(va, vb, tc.op)
		nativeRes := nb.Result(tc.op)
		va.Delete()
		vb.Delete()

		tag := name + "/" + tc.name
		if got, want := nativeRes.Volume(), bridgeRes.Volume(); !approx(got, want) {
			t.Errorf("%s: Volume native=%.12g bridge=%.12g", tag, got, want)
		}
		if got, want := nativeRes.SurfaceArea(), bridgeRes.SurfaceArea(); !approx(got, want) {
			t.Errorf("%s: SurfaceArea native=%.12g bridge=%.12g", tag, got, want)
		}
		if got, want := nativeRes.Genus(), bridgeRes.Genus(); got != want {
			t.Errorf("%s: Genus native=%d bridge=%d", tag, got, want)
		}
		if got, want := nativeRes.NumVert(), bridgeRes.NumVert(); got != want {
			t.Errorf("%s: NumVert native=%d bridge=%d", tag, got, want)
		}
		if got, want := nativeRes.NumTri(), bridgeRes.NumTri(); got != want {
			t.Errorf("%s: NumTri native=%d bridge=%d", tag, got, want)
		}
	}
}

// TestNativeBoolean_VsBridge validates the native Boolean against the C++ bridge
// oracle across diverse shape pairs (axis-aligned, curved, rotated, offset),
// exercising the simple-face and general-triangulation paths and non-axis-aligned
// crossings.
func TestNativeBoolean_VsBridge(t *testing.T) {
	// Axis-aligned cubes overlapping in [0,1]^3.
	compareBooleanVsBridge(t, "cube-cube",
		Cube(Vec3{X: 2, Y: 2, Z: 2}, true), Cube(Vec3{X: 2, Y: 2, Z: 2}, false))

	// Sphere vs offset cube: curved surface + general (>4-edge) faces.
	compareBooleanVsBridge(t, "sphere-cube",
		Sphere(1.0, 32),
		Cube(Vec3{X: 1.5, Y: 1.5, Z: 1.5}, true).Translate(Vec3{X: 0.5, Y: 0.5, Z: 0.5}))

	// Two offset spheres.
	compareBooleanVsBridge(t, "sphere-sphere",
		Sphere(1.0, 24), Sphere(1.0, 24).Translate(Vec3{X: 0.7, Y: 0.3, Z: 0.2}))

	// Rotated cube vs axis-aligned cube: non-axis-aligned crossings.
	compareBooleanVsBridge(t, "rotcube-cube",
		Cube(Vec3{X: 1.6, Y: 1.6, Z: 1.6}, true).Rotate(20, 35, 10),
		Cube(Vec3{X: 2, Y: 2, Z: 2}, true))

	// Cylinder vs cube.
	compareBooleanVsBridge(t, "cylinder-cube",
		Cylinder(2.0, 0.8, 0.8, 32, true), Cube(Vec3{X: 1.5, Y: 1.5, Z: 1.5}, true))
}
