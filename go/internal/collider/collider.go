// Package collider is the Go port of src/collider.h — a linear
// bounding volume hierarchy (LBVH) over an axis-aligned box per leaf.
// Used by Slice, RayCast, MinGap, Minkowski, and the boolean engine
// to filter candidate primitives by AABB overlap in O(log N + k).
//
// Structure mirrors the C++:
//   - Nodes are indexed by a single int. Even indices are leaves
//     (the leaf-id is node/2). Odd indices are internal (the
//     internal-id is (node-1)/2). The root is node 1 (internal 0).
//   - nodeBBox_[node] is each node's AABB.
//   - nodeParent_[node] is the parent node (or -1 for root).
//   - internalChildren_[internal] = {childA, childB}.
//
// The radix-tree construction (Karras 2012) is parallel across
// internal nodes via the leafMorton ordering. Bounding-box
// propagation is parallel across leaves with an atomic counter at
// each internal node to ensure exactly one child climbs into the
// parent. Query (Collisions) is parallel across queries with a
// per-query stack-based DFS through the tree.
package collider

import (
	"math"
	"math/bits"
	"sync/atomic"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
)

// Tunable constants — mirror collider_internal::* in src/collider.h.
const (
	kInitialLength       = 128
	kLengthMultiple      = 4
	kSequentialThreshold = 512
	kRoot                = 1
)

// IsLeaf reports whether node is a leaf (even index).
func IsLeaf(node int) bool { return node%2 == 0 }

// IsInternal reports whether node is an internal node (odd index).
func IsInternal(node int) bool { return node%2 == 1 }

// Node2Internal converts a node index to its internal-id (node-1)/2.
func Node2Internal(node int) int { return (node - 1) / 2 }

// Internal2Node converts an internal-id to a node index 2i+1.
func Internal2Node(i int) int { return i*2 + 1 }

// Node2Leaf converts a node index to its leaf-id node/2.
func Node2Leaf(node int) int { return node / 2 }

// Leaf2Node converts a leaf-id to a node index 2i.
func Leaf2Node(leaf int) int { return leaf * 2 }

// Collider holds the LBVH state. Empty when there are no leaves.
type Collider struct {
	nodeBBox         []geom.Box
	nodeParent       []int32
	internalChildren [][2]int32
}

// Copy returns a deep copy with independent backing arrays, so a refit
// (UpdateBoxes) on the copy never mutates the source. The C++ copy constructor
// copies collider_ wholesale (impl.cpp:675); Impl::Transform relies on this to
// carry a refittable collider into the transformed result.
func (c *Collider) Copy() *Collider {
	return &Collider{
		nodeBBox:         append([]geom.Box(nil), c.nodeBBox...),
		nodeParent:       append([]int32(nil), c.nodeParent...),
		internalChildren: append([][2]int32(nil), c.internalChildren...),
	}
}

// New is the Go port of C++ Collider::Collider(leafBB, leafMorton) (collider.h:267).
// leafMorton must already be sorted ascending — the same precondition as C++,
// whose caller (Impl::Collider via SortFaces) reorders the mesh so leaf == face.
// The Collider does not sort.
func New(leafBB []geom.Box, leafMorton []uint32) *Collider {
	if len(leafBB) != len(leafMorton) {
		panic("collider: leafBB and leafMorton must have equal length")
	}
	c := &Collider{}
	if len(leafBB) == 0 {
		return c
	}
	numNodes := 2*len(leafBB) - 1
	c.nodeBBox = make([]geom.Box, numNodes)
	c.nodeParent = make([]int32, numNodes)
	for i := range c.nodeParent {
		c.nodeParent[i] = -1
	}
	c.internalChildren = make([][2]int32, len(leafBB)-1)
	for i := range c.internalChildren {
		c.internalChildren[i] = [2]int32{-1, -1}
	}
	// Radix tree build — parallel across internal nodes.
	parallel.ForEachN(parallel.AutoPolicy(c.numInternal(), 10000), c.numInternal(),
		func(internal int) {
			c.createRadixTreeStep(internal, leafMorton)
		})
	c.UpdateBoxes(leafBB)
	return c
}

func (c *Collider) numInternal() int { return len(c.internalChildren) }

func (c *Collider) numLeaves() int {
	if len(c.internalChildren) == 0 {
		return 0
	}
	return c.numInternal() + 1
}

// GetBoundingBox returns the root node's AABB, or an empty Box when
// the Collider is empty. Mirrors C++ Collider::GetBoundingBox.
func (c *Collider) GetBoundingBox() geom.Box {
	if len(c.nodeBBox) == 0 {
		return geom.EmptyBox() // C++ returns Box() = {+inf,-inf}, not the origin
	}
	return c.nodeBBox[Internal2Node(0)]
}

// UpdateBoxes is the Go port of C++ Collider::UpdateBoxes. Refresh
// every leaf box and propagate to internal nodes via atomic-counter
// bottom-up sweep.
func (c *Collider) UpdateBoxes(leafBB []geom.Box) {
	// C++ guards with DEBUG_ASSERT(leafBB.size() == NumLeaves()) (collider.h:292)
	// — debug-only, compiled out in release. We must NOT promote it to an
	// unconditional error: NumLeaves() is 0 for a single-leaf collider (the
	// internalChildren_-empty quirk), so the valid single-triangle case has
	// len(leafBB)==1 != numLeaves()==0, and C++ release simply proceeds.
	// Copy leaf boxes into the even-index slots.
	parallel.ForEachN(parallel.AutoPolicy(len(leafBB), 1000), len(leafBB), func(i int) {
		c.nodeBBox[Leaf2Node(i)] = leafBB[i]
	})
	// Per-internal counter: each leaf walks up and the FIRST child to
	// arrive at an internal returns; the SECOND combines child boxes
	// and continues up. AtomicAdd returns the OLD value, so the first
	// arrival sees 0 and returns.
	counter := make([]int32, c.numInternal())
	parallel.ForEachN(parallel.AutoPolicy(c.numInternal(), 1000), c.numLeaves(),
		func(leaf int) {
			c.buildInternalBoxesStep(leaf, counter)
		})
}

func (c *Collider) buildInternalBoxesStep(leaf int, counter []int32) {
	node := Leaf2Node(leaf)
	for {
		node = int(c.nodeParent[node])
		internal := Node2Internal(node)
		// AtomicAdd returns the OLD value (matching C++ AtomicAdd's
		// fetch_add semantic). First arriving child sees 0 → return.
		old := atomic.AddInt32(&counter[internal], 1) - 1
		if old == 0 {
			return
		}
		child1 := c.internalChildren[internal][0]
		child2 := c.internalChildren[internal][1]
		c.nodeBBox[node] = c.nodeBBox[child1].Union(c.nodeBBox[child2])
		if node == kRoot {
			return
		}
	}
}

// Transform is the Go port of C++ Collider::Transform. Applies an
// axis-aligned mat3x4 to every nodeBBox in parallel. The C++ asserts
// the transform is axis-aligned; we don't panic, but it's the
// caller's responsibility (a non-axis-aligned transform leaves the
// boxes conservative-but-loose).
func (c *Collider) Transform(m geom.Mat3x4) {
	parallel.ForEachN(parallel.AutoPolicy(len(c.nodeBBox)), len(c.nodeBBox), func(i int) {
		c.nodeBBox[i] = c.nodeBBox[i].Transform(m)
	})
}

// CollisionsBox is the Go port of the Box-query overload of C++
// Collider::Collisions. For each i in [0, len(queries)), runs a
// stack-based DFS through the tree and invokes record(i, leafIdx)
// for every leaf whose AABB overlaps queries[i]. selfCollision skips
// the case i == leafIdx.
//
// record may be invoked concurrently when parallel is true; the
// caller is responsible for synchronizing any shared mutation. C++
// uses a Recorder<Local>::record(...) pattern with thread-local
// storage; Go callers can wrap their state in a sync.Mutex or use
// sync.Map / channels if they need per-worker locals.
func (c *Collider) CollisionsBox(queries []geom.Box, parallel_ bool, selfCollision bool, record func(queryIdx, leafIdx int)) {
	if len(c.internalChildren) == 0 || len(queries) == 0 {
		return
	}
	policy := parallel.Seq
	if parallel_ {
		policy = parallel.AutoPolicy(len(queries), kSequentialThreshold)
	}
	parallel.ForEachN(policy, len(queries), func(i int) {
		q := queries[i]
		// Empty-box early exit (matches C++ "min.x == +inf" check).
		if math.IsInf(q.Min.X, 1) {
			return
		}
		c.dfs(i, selfCollision, func(b geom.Box) bool { return b.DoesOverlap(q) }, record)
	})
}

// CollisionsPoint is the Go port of the vec3-query overload of
// Collider::Collisions. Same shape as CollisionsBox; uses
// Box.Contains for the leaf-box overlap test.
func (c *Collider) CollisionsPoint(queries []geom.Vec3, parallel_ bool, selfCollision bool, record func(queryIdx, leafIdx int)) {
	if len(c.internalChildren) == 0 || len(queries) == 0 {
		return
	}
	policy := parallel.Seq
	if parallel_ {
		policy = parallel.AutoPolicy(len(queries), kSequentialThreshold)
	}
	parallel.ForEachN(policy, len(queries), func(i int) {
		p := queries[i]
		// The C++ collider tests box.DoesOverlap(query); for a vec3 query that is
		// the XY-projected overload (collider.h:173, common.h:427), not 3D Contains.
		c.dfs(i, selfCollision, func(b geom.Box) bool { return b.DoesOverlapPoint(p) }, record)
	})
}

// dfs is the per-query depth-first search through the radix tree.
// At each internal node we test both children against the overlap
// predicate; if either overlaps and is internal we recurse; if it's
// a leaf and overlaps we record. Stack depth bound: 64 (30 Morton
// bits + 32 index bits used as tiebreakers, plus a few slop).
func (c *Collider) dfs(queryIdx int, selfCollision bool, overlap func(geom.Box) bool, record func(queryIdx, leafIdx int)) {
	var stack [64]int32
	top := -1
	node := int32(kRoot)
	for {
		internal := Node2Internal(int(node))
		c1 := c.internalChildren[internal][0]
		c2 := c.internalChildren[internal][1]
		t1 := c.recordCollision(queryIdx, c1, selfCollision, overlap, record)
		t2 := c.recordCollision(queryIdx, c2, selfCollision, overlap, record)
		if !t1 && !t2 {
			if top < 0 {
				return
			}
			node = stack[top]
			top--
			continue
		}
		if t1 {
			node = c1
			if t2 {
				top++
				stack[top] = c2
			}
		} else {
			node = c2
		}
	}
}

// recordCollision checks one (query, node) pair. If node is a leaf
// and its AABB overlaps the query, records it via the callback.
// Returns true iff node is internal AND overlaps (i.e. should be
// traversed further). Mirrors collider_internal::FindCollision::RecordCollision.
func (c *Collider) recordCollision(queryIdx int, node int32, selfCollision bool, overlap func(geom.Box) bool, record func(queryIdx, leafIdx int)) bool {
	if !overlap(c.nodeBBox[node]) {
		return false
	}
	if IsLeaf(int(node)) {
		leafIdx := Node2Leaf(int(node))
		if !selfCollision || leafIdx != queryIdx {
			record(queryIdx, leafIdx)
		}
		return false
	}
	return true
}

// createRadixTreeStep computes the children of one internal node
// using the Karras 2012 radix-tree algorithm. Mirrors C++
// CreateRadixTree::operator().
func (c *Collider) createRadixTreeStep(internal int, leafMorton []uint32) {
	first := internal
	last := c.rangeEnd(first, leafMorton)
	if first > last {
		first, last = last, first
	}
	split := c.findSplit(first, last, leafMorton)
	var child1, child2 int
	if split == first {
		child1 = Leaf2Node(split)
	} else {
		child1 = Internal2Node(split)
	}
	split++
	if split == last {
		child2 = Leaf2Node(split)
	} else {
		child2 = Internal2Node(split)
	}
	c.internalChildren[internal][0] = int32(child1)
	c.internalChildren[internal][1] = int32(child2)
	node := Internal2Node(internal)
	c.nodeParent[child1] = int32(node)
	c.nodeParent[child2] = int32(node)
}

// prefixLengthMorton is the count of identical highest-order bits in
// (a XOR b). Mirrors C++ PrefixLength(uint32_t, uint32_t).
func prefixLengthMorton(a, b uint32) int {
	x := a ^ b
	if x == 0 {
		return 32
	}
	return bits.LeadingZeros32(x)
}

// prefixLengthIndex mirrors C++ PrefixLength(int, int): if either
// leaf index is out of bounds, returns -1; otherwise the bit-prefix
// length of (morton[i] xor morton[j]), with leaf index as a tie-
// breaker when the morton codes are equal.
func (c *Collider) prefixLengthIndex(i, j int, leafMorton []uint32) int {
	if j < 0 || j >= len(leafMorton) {
		return -1
	}
	if leafMorton[i] == leafMorton[j] {
		return 32 + prefixLengthMorton(uint32(i), uint32(j))
	}
	return prefixLengthMorton(leafMorton[i], leafMorton[j])
}

// rangeEnd determines the end of the morton-prefix range starting at
// i — first walks the direction (+/-1) of equal-prefix extent, then
// exponentially probes to find an upper bound, then binary-searches
// for the precise end. Mirrors C++ CreateRadixTree::RangeEnd.
func (c *Collider) rangeEnd(i int, leafMorton []uint32) int {
	d1 := c.prefixLengthIndex(i, i+1, leafMorton)
	d0 := c.prefixLengthIndex(i, i-1, leafMorton)
	dir := 0
	if d1-d0 > 0 {
		dir = 1
	} else if d1-d0 < 0 {
		dir = -1
	}
	commonPrefix := c.prefixLengthIndex(i, i-dir, leafMorton)
	maxLength := kInitialLength
	for c.prefixLengthIndex(i, i+dir*maxLength, leafMorton) > commonPrefix {
		maxLength *= kLengthMultiple
	}
	length := 0
	for step := maxLength / 2; step > 0; step /= 2 {
		if c.prefixLengthIndex(i, i+dir*(length+step), leafMorton) > commonPrefix {
			length += step
		}
	}
	return i + dir*length
}

// findSplit binary-searches for the split point within [first, last)
// — the furthest leaf that shares more than commonPrefix bits with
// first. Mirrors C++ CreateRadixTree::FindSplit.
func (c *Collider) findSplit(first, last int, leafMorton []uint32) int {
	commonPrefix := c.prefixLengthIndex(first, last, leafMorton)
	split := first
	step := last - first
	for {
		step = (step + 1) >> 1
		newSplit := split + step
		if newSplit < last {
			splitPrefix := c.prefixLengthIndex(first, newSplit, leafMorton)
			if splitPrefix > commonPrefix {
				split = newSplit
			}
		}
		if step <= 1 {
			break
		}
	}
	return split
}

// IsAxisAligned is the Go port of C++ Collider::IsAxisAligned. True
// iff each row of the 3x3 linear part has exactly two zero entries
// (i.e. only one nonzero per row).
func IsAxisAligned(m geom.Mat3x4) bool {
	for row := 0; row < 3; row++ {
		zeros := 0
		for col := 0; col < 3; col++ {
			if m[col][row] == 0 {
				zeros++
			}
		}
		if zeros != 2 {
			return false
		}
	}
	return true
}
