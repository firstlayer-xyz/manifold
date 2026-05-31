package manifold

import (
	"math"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/internal/cppref"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// corpusEntry is one polygon test from the manifold corpus files
// (test/polygons/*.txt), in the format documented in test/polygon_test.cpp:
//
//	name expectedNumTri epsilon numPolys
//	  numPoints  (x y)*numPoints   (repeated numPolys times)
type corpusEntry struct {
	name    string
	epsilon float64
	numTri  int
	polys   [][]geom.Vec2
}

// parseCorpus tokenizes a corpus file on any whitespace (matching the C++
// std::ifstream >> reader) and parses every test entry.
func parseCorpus(t *testing.T, path string) []corpusEntry {
	t.Helper()
	data, err := os.ReadFile(path)
	if err != nil {
		t.Fatalf("read corpus %s: %v", path, err)
	}
	toks := strings.Fields(string(data))
	i := 0
	atof := func() float64 {
		v, err := strconv.ParseFloat(toks[i], 64)
		if err != nil {
			t.Fatalf("%s: bad float %q at token %d: %v", path, toks[i], i, err)
		}
		i++
		return v
	}
	atoi := func() int {
		v, err := strconv.Atoi(toks[i])
		if err != nil {
			t.Fatalf("%s: bad int %q at token %d: %v", path, toks[i], i, err)
		}
		i++
		return v
	}
	var entries []corpusEntry
	for i < len(toks) {
		name := toks[i]
		i++
		numTri := atoi()
		epsilon := atof()
		numPolys := atoi()
		polys := make([][]geom.Vec2, numPolys)
		for p := 0; p < numPolys; p++ {
			numPoints := atoi()
			poly := make([]geom.Vec2, numPoints)
			for j := 0; j < numPoints; j++ {
				x := atof()
				y := atof()
				poly[j] = geom.Vec2{X: x, Y: y}
			}
			polys[p] = poly
		}
		entries = append(entries, corpusEntry{name, epsilon, numTri, polys})
	}
	return entries
}

// turn180 negates every coordinate (mirrors Turn180 in polygon_test.cpp): a
// 180° rotation that must not change the triangle count.
func turn180(polys [][]geom.Vec2) [][]geom.Vec2 {
	out := make([][]geom.Vec2, len(polys))
	for i, poly := range polys {
		p := make([]geom.Vec2, len(poly))
		for j, v := range poly {
			p[j] = geom.Vec2{X: -v.X, Y: -v.Y}
		}
		out[i] = p
	}
	return out
}

// duplicate appends an x-shifted copy of every contour (mirrors Duplicate in
// polygon_test.cpp): two disjoint components must yield twice the triangles.
func duplicate(polys [][]geom.Vec2) [][]geom.Vec2 {
	xMin, xMax := math.Inf(1), math.Inf(-1)
	for _, poly := range polys {
		for _, v := range poly {
			xMin = math.Min(xMin, v.X)
			xMax = math.Max(xMax, v.X)
		}
	}
	shift := xMax - xMin
	out := make([][]geom.Vec2, 0, 2*len(polys))
	out = append(out, polys...)
	for _, poly := range polys {
		p := make([]geom.Vec2, len(poly))
		for j, v := range poly {
			p[j] = geom.Vec2{X: v.X + shift, Y: v.Y}
		}
		out = append(out, p)
	}
	return out
}

// TestTriangulateCorpus_VsCpp runs the native triangulator over the manifold
// polygon corpus (the same fixtures the C++ PolygonTest uses) and asserts it
// produces the curated expected triangle count — the exact bar of C++ TestPoly
// (Basic / Turn180 / Duplicate) — plus that it agrees with the cppref. The
// large zebra fixtures are gated behind -short because the increment-6
// brute-force collider is O(n^2) (the kd-tree lands in increment 9).
func TestTriangulateCorpus_VsCpp(t *testing.T) {
	small := []string{"polygon_corpus.txt", "sponge.txt"}
	large := []string{"zebra.txt", "zebra3.txt"}

	run := func(t *testing.T, file string, full bool) {
		entries := parseCorpus(t, filepath.Join("..", "test", "polygons", file))
		for _, e := range entries {
			t.Run(e.name, func(t *testing.T) {
				got := nativeFlat(e.polys, e.epsilon)
				if len(got)/3 != e.numTri {
					t.Errorf("Basic: numTri %d != expected %d", len(got)/3, e.numTri)
				}
				if want := cppref.Triangulate(e.polys, e.epsilon); len(got) != len(want) {
					t.Errorf("Basic: count %d != bridge %d", len(got)/3, len(want)/3)
				} else if !sameTriangulation(got, want) {
					t.Logf("%s: differs from bridge as a set (both same count)", e.name)
				}
				if !full {
					return
				}
				if n := len(nativeFlat(turn180(e.polys), e.epsilon)) / 3; n != e.numTri {
					t.Errorf("Turn180: numTri %d != expected %d", n, e.numTri)
				}
				if n := len(nativeFlat(duplicate(e.polys), e.epsilon)) / 3; n != 2*e.numTri {
					t.Errorf("Duplicate: numTri %d != expected %d", n, 2*e.numTri)
				}
			})
		}
	}

	for _, file := range small {
		t.Run(file, func(t *testing.T) { run(t, file, true) })
	}
	if testing.Short() {
		t.Log("skipping zebra/zebra3 (O(n^2) collider) in -short mode")
		return
	}
	for _, file := range large {
		t.Run(file, func(t *testing.T) { run(t, file, false) })
	}
}
