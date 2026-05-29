package triangulate

import (
	"math"
	"sort"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// tree2d is the Go port of src/tree2d.{h,cpp}: a k=2 kd-tree with alternating
// x/y partition, built by recursively stable-sorting in place. The points slice
// is reordered so that the median at each level is the pivot; each PolyVert
// keeps its Idx (an index back into the owning IdxCollider's itr slice), so the
// reordering is lossless. Used by EarClip::EarCost to find, for each ear, the
// subset of verts inside the ear's bounding box.

// buildTwoDTreeImpl is the Go port of BuildTwoDTreeImpl (tree2d.cpp:39). It
// stable-sorts points on x (or y), then recurses on the two halves with the
// axis flipped. Recursive sorting is not the most efficient but is simple and
// guaranteed to produce a balanced tree. Sub-slices share the backing array, so
// the sort is in place — the Go analog of C++ VecView.
func buildTwoDTreeImpl(points []PolyVert, sortX bool) {
	if sortX {
		sort.SliceStable(points, func(i, j int) bool { return points[i].Pos.X < points[j].Pos.X })
	} else {
		sort.SliceStable(points, func(i, j int) bool { return points[i].Pos.Y < points[j].Pos.Y })
	}
	if len(points) < 2 {
		return
	}
	buildTwoDTreeImpl(points[:len(points)/2], !sortX)
	buildTwoDTreeImpl(points[len(points)/2+1:], !sortX)
}

// buildTwoDTree is the Go port of BuildTwoDTree (tree2d.cpp:55): builds the tree
// in place, skipping the work entirely for small point sets.
func buildTwoDTree(points []PolyVert) {
	// don't even bother...
	if len(points) <= 8 {
		return
	}
	buildTwoDTreeImpl(points, true)
}

// queryTwoDTree is the Go port of QueryTwoDTree (tree2d.h:28): invoke f for
// every point inside r, walking the (implicit) tree with an explicit stack and
// pruning subtrees whose half-space cannot overlap r. f is called exactly once
// per point whose pos is Contained in r, regardless of traversal order.
func queryTwoDTree(points []PolyVert, r geom.Rect, f func(PolyVert)) {
	if len(points) <= 8 {
		for _, p := range points {
			if r.Contains(p.Pos) {
				f(p)
			}
		}
		return
	}
	current := geom.Rect{
		Min: geom.Vec2{X: math.Inf(-1), Y: math.Inf(-1)},
		Max: geom.Vec2{X: math.Inf(1), Y: math.Inf(1)},
	}

	level := 0
	currentView := points
	var rectStack [64]geom.Rect
	var viewStack [64][]PolyVert
	var levelStack [64]int
	stackPointer := 0

	for {
		if len(currentView) <= 8 {
			for _, p := range currentView {
				if r.Contains(p.Pos) {
					f(p)
				}
			}
			stackPointer--
			if stackPointer < 0 {
				break
			}
			level = levelStack[stackPointer]
			currentView = viewStack[stackPointer]
			current = rectStack[stackPointer]
			continue
		}

		// these are conceptual left/right trees
		left := current
		right := current
		middle := currentView[len(currentView)/2]
		if level%2 == 0 {
			left.Max.X = middle.Pos.X
			right.Min.X = middle.Pos.X
		} else {
			left.Max.Y = middle.Pos.Y
			right.Min.Y = middle.Pos.Y
		}

		if r.Contains(middle.Pos) {
			f(middle)
		}
		if left.DoesOverlap(r) {
			if right.DoesOverlap(r) {
				rectStack[stackPointer] = right
				viewStack[stackPointer] = currentView[len(currentView)/2+1:]
				levelStack[stackPointer] = level + 1
				stackPointer++
			}
			current = left
			currentView = currentView[:len(currentView)/2]
			level++
		} else {
			current = right
			currentView = currentView[len(currentView)/2+1:]
			level++
		}
	}
}
