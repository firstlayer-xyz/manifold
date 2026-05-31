package manifold

// Go-vs-C++ performance baseline. Each operation has a _Go benchmark (native Go) and
// a _Cpp benchmark (the cppref C++ reference) on identical inputs, so the ratio shows
// where the native port stands. Operands are built once before the timer; the timed
// loop runs only the operation. C++ result handles are deleted each iteration so the
// benchmark measures the op, not handle accumulation.

import (
	"testing"

	"github.com/firstlayer-xyz/manifold/go/internal/cppref"
	"github.com/firstlayer-xyz/manifold/go/internal/handle"
)

const benchSeg = 64 // sphere circular segments for the main workloads

// eval forces a C++ CSG result to actually compute. The C++ manifold is LAZY —
// boolean/refine/etc. build a deferred CSG node and only evaluate when queried —
// whereas the Go port is EAGER (the op runs immediately). NumTri triggers leaf
// evaluation, so the _Cpp benchmarks measure the same work the _Go ones do.
func eval(h *handle.Manifold) *handle.Manifold {
	_ = cppref.NumTri(h)
	return h
}

// ---- Union (two overlapping spheres) ----

func BenchmarkUnion_Go(b *testing.B) {
	a := Sphere(1, benchSeg)
	c := Sphere(1, benchSeg).Translate(Vec3{X: 0.8})
	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_ = a.Union(c)
	}
}

func BenchmarkUnion_Cpp(b *testing.B) {
	ha := cppref.Sphere(1, benchSeg)
	hs := cppref.Sphere(1, benchSeg)
	hb := cppref.Translate(hs, Vec3{X: 0.8})
	cppref.DeleteManifold(hs)
	defer cppref.DeleteManifold(ha)
	defer cppref.DeleteManifold(hb)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		cppref.DeleteManifold(eval(cppref.Union(ha, hb)))
	}
}

// ---- Difference (two overlapping spheres) ----

func BenchmarkDifference_Go(b *testing.B) {
	a := Sphere(1, benchSeg)
	c := Sphere(1, benchSeg).Translate(Vec3{X: 0.8})
	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_ = a.Difference(c)
	}
}

func BenchmarkDifference_Cpp(b *testing.B) {
	ha := cppref.Sphere(1, benchSeg)
	hs := cppref.Sphere(1, benchSeg)
	hb := cppref.Translate(hs, Vec3{X: 0.8})
	cppref.DeleteManifold(hs)
	defer cppref.DeleteManifold(ha)
	defer cppref.DeleteManifold(hb)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		cppref.DeleteManifold(eval(cppref.Difference(ha, hb)))
	}
}

// ---- Sphere construction ----

func BenchmarkSphere_Go(b *testing.B) {
	b.ReportAllocs()
	for i := 0; i < b.N; i++ {
		_ = Sphere(1, benchSeg)
	}
}

func BenchmarkSphere_Cpp(b *testing.B) {
	for i := 0; i < b.N; i++ {
		cppref.DeleteManifold(cppref.Sphere(1, benchSeg))
	}
}

// ---- Refine (subdivision) ----

func BenchmarkRefine_Go(b *testing.B) {
	a := Sphere(1, 32)
	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_ = a.Refine(4)
	}
}

func BenchmarkRefine_Cpp(b *testing.B) {
	ha := cppref.Sphere(1, 32)
	defer cppref.DeleteManifold(ha)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		cppref.DeleteManifold(eval(cppref.Refine(ha, 4)))
	}
}

// ---- Hull (convex hull of a sphere's vertices) ----

func BenchmarkHull_Go(b *testing.B) {
	a := Sphere(1, benchSeg)
	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_ = a.Hull()
	}
}

func BenchmarkHull_Cpp(b *testing.B) {
	ha := cppref.Sphere(1, benchSeg)
	defer cppref.DeleteManifold(ha)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		cppref.DeleteManifold(eval(cppref.Hull(ha)))
	}
}

// ---- CalculateNormals ----

func BenchmarkCalculateNormals_Go(b *testing.B) {
	a := Sphere(1, benchSeg)
	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_ = a.CalculateNormals(0, 60)
	}
}

func BenchmarkCalculateNormals_Cpp(b *testing.B) {
	ha := cppref.Sphere(1, benchSeg)
	defer cppref.DeleteManifold(ha)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		cppref.DeleteManifold(eval(cppref.CalculateNormals(ha, 0, 60)))
	}
}

// ---- MinkowskiSum (low-res operands — this op is expensive) ----

func BenchmarkMinkowskiSum_Go(b *testing.B) {
	a := Sphere(1, 16)
	c := Cube(Vec3{X: 0.3, Y: 0.3, Z: 0.3}, true)
	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_ = a.MinkowskiSum(c)
	}
}

func BenchmarkMinkowskiSum_Cpp(b *testing.B) {
	ha := cppref.Sphere(1, 16)
	hc := cppref.Cube(0.3, 0.3, 0.3, true)
	defer cppref.DeleteManifold(ha)
	defer cppref.DeleteManifold(hc)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		cppref.DeleteManifold(eval(cppref.MinkowskiSum(ha, hc)))
	}
}
