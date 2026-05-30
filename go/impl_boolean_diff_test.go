package manifold

import (
	"math"
	"runtime"
	"testing"
)

// TestNativeBoolean_VsBridge is the differential test: the native Boolean
// (nativeBoolean3.Result -- assemble/face2Tri/reorder + the Impl finalize tail)
// vs the bridge Boolean (the unmodified C++ oracle), for two overlapping cubes
// across all three ops. Geometry must match the bridge AND the analytic volumes
// (union=15, subtract=7, intersect=1 for [-1,1]^3 vs [0,2]^3).
func TestNativeBoolean_VsBridge(t *testing.T) {
	ma := Cube(Vec3{X: 2, Y: 2, Z: 2}, true)  // [-1,1]^3, vol 8
	mb := Cube(Vec3{X: 2, Y: 2, Z: 2}, false) // [0,2]^3, vol 8, overlap [0,1]^3 vol 1
	defer runtime.KeepAlive(ma)
	defer runtime.KeepAlive(mb)

	cases := []struct {
		name   string
		op     OpType
		volume float64
	}{
		{"Add", OpAdd, 15},
		{"Subtract", OpSubtract, 7},
		{"Intersect", OpIntersect, 1},
	}
	approx := func(a, b float64) bool { return math.Abs(a-b) <= 1e-9*(1+math.Abs(b)) }

	for _, tc := range cases {
		bridgeRes := ma.Boolean(mb, tc.op)

		va := getImpl(ma)
		vb := getImpl(mb)
		nb := newNativeBoolean3(va, vb, tc.op)
		nativeRes := nb.Result(tc.op)
		va.Delete()
		vb.Delete()

		// Analytic volume.
		if v := nativeRes.Volume(); !approx(v, tc.volume) {
			t.Errorf("%s: native Volume=%.12g, want %.0f", tc.name, v, tc.volume)
		}
		// Match the bridge oracle.
		if got, want := nativeRes.Volume(), bridgeRes.Volume(); !approx(got, want) {
			t.Errorf("%s: Volume native=%.12g bridge=%.12g", tc.name, got, want)
		}
		if got, want := nativeRes.SurfaceArea(), bridgeRes.SurfaceArea(); !approx(got, want) {
			t.Errorf("%s: SurfaceArea native=%.12g bridge=%.12g", tc.name, got, want)
		}
		if got, want := nativeRes.Genus(), bridgeRes.Genus(); got != want {
			t.Errorf("%s: Genus native=%d bridge=%d", tc.name, got, want)
		}
		if got, want := nativeRes.NumVert(), bridgeRes.NumVert(); got != want {
			t.Errorf("%s: NumVert native=%d bridge=%d", tc.name, got, want)
		}
		if got, want := nativeRes.NumTri(), bridgeRes.NumTri(); got != want {
			t.Errorf("%s: NumTri native=%d bridge=%d", tc.name, got, want)
		}
	}
}
