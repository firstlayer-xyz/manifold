package manifold

import (
	"math"
	"sort"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/internal/cppref"
)

// TestImplStorageRoundTrip validates the marshalling seam: native implStorage
// marshalled out to a C++ Impl and back must reproduce the same mesh (geometry +
// properties + counts). Exercises a property-carrying, transformed mesh.
func TestImplStorageRoundTrip(t *testing.T) {
	m := Sphere(1.0, 20).CalculateNormals(3, 60).Translate(Vec3{X: 0.3, Y: 0.2})

	// refHandle marshals s -> a C++ handle (marshalImplStorageToBridge); reading it
	// back via marshalImplStorageFromBridge round-trips through the full bridge seam.
	rh := m.refHandle()
	defer cppref.DeleteManifold(rh)
	bi := cppref.GetImpl(rh)
	defer bi.Delete()
	s := marshalImplStorageFromBridge(bi)

	// Adopt s as the mutable impl's native storage; ToManifold publishes it.
	mi := newImpl()
	defer mi.Delete()
	mi.s = s
	m2 := mi.ToManifold()

	approx := func(a, b float64) bool { return math.Abs(a-b) <= 1e-9*(1+math.Abs(b)) }
	if got, want := m2.Volume(), m.Volume(); !approx(got, want) {
		t.Errorf("Volume: round-trip=%.12g orig=%.12g", got, want)
	}
	if got, want := m2.SurfaceArea(), m.SurfaceArea(); !approx(got, want) {
		t.Errorf("SurfaceArea: round-trip=%.12g orig=%.12g", got, want)
	}
	if got, want := m2.Genus(), m.Genus(); got != want {
		t.Errorf("Genus: round-trip=%d orig=%d", got, want)
	}
	if got, want := m2.NumVert(), m.NumVert(); got != want {
		t.Errorf("NumVert: round-trip=%d orig=%d", got, want)
	}
	if got, want := m2.NumTri(), m.NumTri(); got != want {
		t.Errorf("NumTri: round-trip=%d orig=%d", got, want)
	}
	if got, want := m2.NumProp(), m.NumProp(); got != want {
		t.Errorf("NumProp: round-trip=%d orig=%d", got, want)
	}
	if got, want := m2.NumPropVert(), m.NumPropVert(); got != want {
		t.Errorf("NumPropVert: round-trip=%d orig=%d", got, want)
	}
	// Properties as an order-insensitive multiset.
	props := func(mm *Manifold) []float64 {
		impl := getImpl(mm)
		defer impl.Delete()
		p := append([]float64(nil), impl.Properties()...)
		sort.Float64s(p)
		return p
	}
	p1, p2 := props(m), props(m2)
	if len(p1) != len(p2) {
		t.Fatalf("properties len: orig=%d round-trip=%d", len(p1), len(p2))
	}
	for i := range p1 {
		if !approx(p1[i], p2[i]) {
			t.Errorf("sorted properties[%d]: orig=%.10g round-trip=%.10g", i, p1[i], p2[i])
			break
		}
	}
}
