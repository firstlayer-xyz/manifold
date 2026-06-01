package manifold

// Scaling sweep: how does the Go-vs-C++ gap move as geometry complexity grows?
// Sphere(1, seg) has ~O(seg^2) triangles, so sweeping seg sweeps mesh complexity.
// Run the _Go and _Cpp variants and compare the ratio at each seg to see whether
// the gap narrows (parallelism amortizes / fixed overhead shrinks), holds (pure
// constant-factor codegen+GC), or widens (GC byte-churn grows with size) at scale.
//
// REQUIRES a Release (-O3) libmanifold (see perf_bench_test.go header).

import (
	"fmt"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/internal/cppref"
)

// scalingSegs spans ~16x in segments => ~256x in triangle count.
var scalingSegs = []int{32, 64, 128, 256, 512}

// BenchmarkScaleTris reports the triangle count of one operand sphere at each seg
// (not a real benchmark — a one-shot size table for interpreting the sweep).
func BenchmarkScaleTris(b *testing.B) {
	for _, seg := range scalingSegs {
		n := Sphere(1, seg).NumTri()
		b.Run(fmt.Sprintf("seg%d", seg), func(b *testing.B) {
			b.Logf("seg=%d operand tris=%d", seg, n)
		})
	}
}

func BenchmarkUnionScaling_Go(b *testing.B) {
	for _, seg := range scalingSegs {
		a := Sphere(1, seg)
		c := Sphere(1, seg).Translate(Vec3{X: 0.8})
		b.Run(fmt.Sprintf("seg%d", seg), func(b *testing.B) {
			b.ReportAllocs()
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				_ = a.Union(c)
			}
		})
	}
}

func BenchmarkUnionScaling_Cpp(b *testing.B) {
	for _, seg := range scalingSegs {
		ha := cppref.Sphere(1, seg)
		hs := cppref.Sphere(1, seg)
		hb := cppref.Translate(hs, Vec3{X: 0.8})
		cppref.DeleteManifold(hs)
		b.Run(fmt.Sprintf("seg%d", seg), func(b *testing.B) {
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				cppref.DeleteManifold(eval(cppref.Union(ha, hb)))
			}
		})
		cppref.DeleteManifold(ha)
		cppref.DeleteManifold(hb)
	}
}
