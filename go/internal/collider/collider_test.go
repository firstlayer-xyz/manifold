package collider

import (
	"math"
	"sort"
	"sync"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// helper: build a Collider from a slice of leaf boxes (the bBox enclosing
// all of them is used to compute Morton codes, and the leaves are
// stable-sorted by morton before being passed to New — matching how
// the SortGeometry pipeline feeds the C++ Collider).
func buildFrom(leaves []geom.Box) (*Collider, []int) {
	if len(leaves) == 0 {
		return New(nil, nil), nil
	}
	// World bounding box.
	world := leaves[0]
	for _, b := range leaves[1:] {
		world = world.Union(b)
	}
	// Per-leaf centroid Morton code.
	codes := make([]uint32, len(leaves))
	for i, b := range leaves {
		codes[i] = geom.MortonCode(b.Center(), world)
	}
	// Sort leaves by morton, preserving original index for the test.
	order := make([]int, len(leaves))
	for i := range order {
		order[i] = i
	}
	sort.SliceStable(order, func(a, b int) bool {
		return codes[order[a]] < codes[order[b]]
	})
	sortedBoxes := make([]geom.Box, len(leaves))
	sortedCodes := make([]uint32, len(leaves))
	for newIdx, oldIdx := range order {
		sortedBoxes[newIdx] = leaves[oldIdx]
		sortedCodes[newIdx] = codes[oldIdx]
	}
	return New(sortedBoxes, sortedCodes), order
}

// TestCollider_Empty: zero leaves builds without panic, bounding box
// is empty, queries no-op.
func TestCollider_Empty(t *testing.T) {
	c := New(nil, nil)
	if c.numLeaves() != 0 {
		t.Errorf("empty Collider numLeaves = %d", c.numLeaves())
	}
	// C++ Collider::GetBoundingBox returns Box() = {+inf,-inf} (the empty box)
	// for an empty collider, NOT the origin box.
	b := c.GetBoundingBox()
	if b != geom.EmptyBox() {
		t.Errorf("empty Collider GetBoundingBox = %+v, want EmptyBox", b)
	}
	called := false
	c.CollisionsBox([]geom.Box{{Min: geom.Vec3{}, Max: geom.Vec3{X: 1, Y: 1, Z: 1}}},
		true, false, func(int, int) { called = true })
	if called {
		t.Error("CollisionsBox on empty Collider should not invoke recorder")
	}
}

// TestCollider_GetBoundingBox builds a Collider from disjoint boxes
// and verifies the root box encloses all of them.
func TestCollider_GetBoundingBox(t *testing.T) {
	leaves := []geom.Box{
		{Min: geom.Vec3{X: 0, Y: 0, Z: 0}, Max: geom.Vec3{X: 1, Y: 1, Z: 1}},
		{Min: geom.Vec3{X: 2, Y: 0, Z: 0}, Max: geom.Vec3{X: 3, Y: 1, Z: 1}},
		{Min: geom.Vec3{X: 0, Y: 2, Z: 0}, Max: geom.Vec3{X: 1, Y: 3, Z: 1}},
		{Min: geom.Vec3{X: 0, Y: 0, Z: 2}, Max: geom.Vec3{X: 1, Y: 1, Z: 3}},
	}
	c, _ := buildFrom(leaves)
	got := c.GetBoundingBox()
	want := geom.Box{Min: geom.Vec3{}, Max: geom.Vec3{X: 3, Y: 3, Z: 3}}
	if got != want {
		t.Errorf("GetBoundingBox = %+v want %+v", got, want)
	}
}

// TestCollider_BoxQueries: build a grid of unit boxes and query
// against a box that overlaps a known subset. Verify the recorded
// leaf indices match expectation.
func TestCollider_BoxQueries(t *testing.T) {
	// 4×4×4 grid of unit boxes.
	var leaves []geom.Box
	for x := 0; x < 4; x++ {
		for y := 0; y < 4; y++ {
			for z := 0; z < 4; z++ {
				leaves = append(leaves, geom.Box{
					Min: geom.Vec3{X: float64(x), Y: float64(y), Z: float64(z)},
					Max: geom.Vec3{X: float64(x + 1), Y: float64(y + 1), Z: float64(z + 1)},
				})
			}
		}
	}
	c, order := buildFrom(leaves)

	// Query: a box that overlaps the corner 2×2×2 cube of the grid.
	q := geom.Box{Min: geom.Vec3{X: 0.5, Y: 0.5, Z: 0.5}, Max: geom.Vec3{X: 2.5, Y: 2.5, Z: 2.5}}
	var mu sync.Mutex
	hits := map[int]bool{}
	c.CollisionsBox([]geom.Box{q}, true, false, func(qi, li int) {
		mu.Lock()
		hits[order[li]] = true
		mu.Unlock()
	})

	// Expected hits: all leaves whose Box overlaps q.
	want := map[int]bool{}
	for i, b := range leaves {
		if b.DoesOverlap(q) {
			want[i] = true
		}
	}
	if len(hits) != len(want) {
		t.Fatalf("hits = %d want %d", len(hits), len(want))
	}
	for k := range want {
		if !hits[k] {
			t.Errorf("missing leaf %d in hits", k)
		}
	}
}

// TestCollider_PointQueries verifies the Vec3-query overload returns
// the expected leaf indices for points placed at known leaf centers.
func TestCollider_PointQueries(t *testing.T) {
	leaves := []geom.Box{
		{Min: geom.Vec3{X: 0, Y: 0, Z: 0}, Max: geom.Vec3{X: 1, Y: 1, Z: 1}},
		{Min: geom.Vec3{X: 1, Y: 0, Z: 0}, Max: geom.Vec3{X: 2, Y: 1, Z: 1}},
		{Min: geom.Vec3{X: 2, Y: 0, Z: 0}, Max: geom.Vec3{X: 3, Y: 1, Z: 1}},
	}
	c, order := buildFrom(leaves)
	// Point at center of leaf-0.
	p := geom.Vec3{X: 0.5, Y: 0.5, Z: 0.5}
	var mu sync.Mutex
	hits := []int{}
	c.CollisionsPoint([]geom.Vec3{p}, true, false, func(qi, li int) {
		mu.Lock()
		hits = append(hits, order[li])
		mu.Unlock()
	})
	// Point lies strictly inside leaf-0; may also lie on leaf-1's
	// boundary depending on numerical bounds — accept any hit set
	// that contains 0.
	found0 := false
	for _, h := range hits {
		if h == 0 {
			found0 = true
		}
	}
	if !found0 {
		t.Errorf("expected leaf 0 in hits, got %v", hits)
	}
}

// TestCollider_SelfCollision builds a Collider over overlapping boxes
// and runs queries with selfCollision=true. The query box at index i
// is the same as leaf i, so without selfCollision we'd record (i,i)
// pairs; selfCollision must skip those.
func TestCollider_SelfCollision(t *testing.T) {
	leaves := []geom.Box{
		{Min: geom.Vec3{X: 0, Y: 0, Z: 0}, Max: geom.Vec3{X: 2, Y: 2, Z: 2}},
		{Min: geom.Vec3{X: 1, Y: 1, Z: 1}, Max: geom.Vec3{X: 3, Y: 3, Z: 3}},
		{Min: geom.Vec3{X: 2, Y: 2, Z: 2}, Max: geom.Vec3{X: 4, Y: 4, Z: 4}},
	}
	c, order := buildFrom(leaves)
	// Queries in original order.
	queries := append([]geom.Box(nil), leaves...)
	// Build a fresh leaf-id query slice in MORTON-SORTED order, so
	// the queryIdx index alignment matches leafIdx alignment in the
	// Collider's index space — selfCollision needs queryIdx == leafIdx
	// to skip.
	mortonQueries := make([]geom.Box, len(leaves))
	for newIdx, oldIdx := range order {
		mortonQueries[newIdx] = queries[oldIdx]
	}
	var mu sync.Mutex
	hits := map[[2]int]bool{}
	c.CollisionsBox(mortonQueries, true, true, func(qi, li int) {
		if qi == li {
			t.Errorf("self-collision pair (%d,%d) should have been skipped", qi, li)
		}
		mu.Lock()
		hits[[2]int{qi, li}] = true
		mu.Unlock()
	})
	// Sanity: leaves 0+1, 1+2 overlap. Each overlapping pair appears
	// twice (once with each as query, once as leaf).
	if len(hits) < 4 {
		t.Errorf("expected at least 4 overlap-pair hits, got %d: %v", len(hits), hits)
	}
}

// TestCollider_UpdateBoxes shifts every leaf by a fixed translation,
// calls UpdateBoxes, and verifies the root bounding box updates.
func TestCollider_UpdateBoxes(t *testing.T) {
	leaves := []geom.Box{
		{Min: geom.Vec3{X: 0, Y: 0, Z: 0}, Max: geom.Vec3{X: 1, Y: 1, Z: 1}},
		{Min: geom.Vec3{X: 2, Y: 2, Z: 2}, Max: geom.Vec3{X: 3, Y: 3, Z: 3}},
	}
	c, order := buildFrom(leaves)
	// Translate every leaf by (10, 0, 0).
	t1 := geom.Vec3{X: 10, Y: 0, Z: 0}
	newLeaves := make([]geom.Box, len(leaves))
	for i, oldIdx := range order {
		newLeaves[i] = leaves[oldIdx].Translate(t1)
	}
	c.UpdateBoxes(newLeaves)
	got := c.GetBoundingBox()
	want := geom.Box{
		Min: geom.Vec3{X: 10, Y: 0, Z: 0},
		Max: geom.Vec3{X: 13, Y: 3, Z: 3},
	}
	if got != want {
		t.Errorf("after UpdateBoxes: root = %+v want %+v", got, want)
	}
}

// TestCollider_Transform builds a Collider, runs Transform with an
// axis-aligned translation matrix, and verifies the root box shifts.
func TestCollider_Transform(t *testing.T) {
	leaves := []geom.Box{
		{Min: geom.Vec3{X: 0, Y: 0, Z: 0}, Max: geom.Vec3{X: 1, Y: 1, Z: 1}},
		{Min: geom.Vec3{X: 2, Y: 2, Z: 2}, Max: geom.Vec3{X: 3, Y: 3, Z: 3}},
	}
	c, _ := buildFrom(leaves)
	// Identity rotation + translation (5, 7, 11).
	m := geom.Mat3x4{
		{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {5, 7, 11},
	}
	if !IsAxisAligned(m) {
		t.Fatal("test transform must be axis-aligned")
	}
	c.Transform(m)
	got := c.GetBoundingBox()
	want := geom.Box{
		Min: geom.Vec3{X: 5, Y: 7, Z: 11},
		Max: geom.Vec3{X: 8, Y: 10, Z: 14},
	}
	if got != want {
		t.Errorf("after Transform: root = %+v want %+v", got, want)
	}
}

// TestCollider_EmptyBoxQuery: a query with an infinite-min box (the
// C++ sentinel for "empty box, skip me") must not produce any hits.
func TestCollider_EmptyBoxQuery(t *testing.T) {
	leaves := []geom.Box{
		{Min: geom.Vec3{}, Max: geom.Vec3{X: 1, Y: 1, Z: 1}},
		{Min: geom.Vec3{X: 2, Y: 0, Z: 0}, Max: geom.Vec3{X: 3, Y: 1, Z: 1}},
	}
	c, _ := buildFrom(leaves)
	empty := geom.Box{
		Min: geom.Vec3{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)},
		Max: geom.Vec3{X: math.Inf(-1), Y: math.Inf(-1), Z: math.Inf(-1)},
	}
	c.CollisionsBox([]geom.Box{empty}, true, false, func(qi, li int) {
		t.Errorf("empty-box query produced hit (q=%d l=%d)", qi, li)
	})
}

// TestIsAxisAligned exercises the static helper.
func TestIsAxisAligned(t *testing.T) {
	ident := geom.Mat3x4{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 0}}
	if !IsAxisAligned(ident) {
		t.Error("identity should be axis-aligned")
	}
	rotZ45 := geom.Mat3x4{{0.707, 0.707, 0}, {-0.707, 0.707, 0}, {0, 0, 1}, {0, 0, 0}}
	if IsAxisAligned(rotZ45) {
		t.Error("45-deg Z rotation should NOT be axis-aligned")
	}
	// Permutation (swap X and Y axes) is axis-aligned.
	swap := geom.Mat3x4{{0, 1, 0}, {1, 0, 0}, {0, 0, 1}, {0, 0, 0}}
	if !IsAxisAligned(swap) {
		t.Error("axis-swap should be axis-aligned")
	}
}

// TestCollider_SingleLeaf guards that constructing a 1-leaf collider does NOT
// panic. Before the fix, New -> UpdateBoxes panicked because numLeaves() is 0
// for a single leaf (the C++ NumLeaves internalChildren_-empty quirk) while
// len(leafBB)==1, and the Go promoted the C++ debug-only size DEBUG_ASSERT to an
// unconditional panic. C++ release proceeds; so must we. (GetBoundingBox and
// queries on a single-leaf collider read internal node 0, which does not exist —
// that is out-of-bounds in BOTH C++ and Go, so it is not exercised here.)
func TestCollider_SingleLeaf(t *testing.T) {
	box := geom.Box{Min: geom.Vec3{X: 0, Y: 0, Z: 0}, Max: geom.Vec3{X: 1, Y: 1, Z: 1}}
	c := New([]geom.Box{box}, []uint32{0}) // must not panic
	if c == nil {
		t.Fatal("New returned nil for single-leaf collider")
	}
}

// TestCollider_Copy verifies Copy is a deep copy: refitting the copy (UpdateBoxes
// with translated leaves) must not mutate the source's boxes.
func TestCollider_Copy(t *testing.T) {
	leaves := []geom.Box{
		{Min: geom.Vec3{X: 0, Y: 0, Z: 0}, Max: geom.Vec3{X: 1, Y: 1, Z: 1}},
		{Min: geom.Vec3{X: 2, Y: 2, Z: 2}, Max: geom.Vec3{X: 3, Y: 3, Z: 3}},
	}
	c, order := buildFrom(leaves)
	srcBox := c.GetBoundingBox()

	cp := c.Copy()
	// Refit the copy with translated leaves (in the collider's leaf order).
	moved := make([]geom.Box, len(leaves))
	for i, oldIdx := range order {
		moved[i] = leaves[oldIdx].Translate(geom.Vec3{X: 100})
	}
	cp.UpdateBoxes(moved)

	if c.GetBoundingBox() != srcBox {
		t.Errorf("source bbox mutated by copy refit: %+v != %+v", c.GetBoundingBox(), srcBox)
	}
	if cp.GetBoundingBox() == srcBox {
		t.Errorf("copy bbox not updated by refit: %+v", cp.GetBoundingBox())
	}
}
