package manifold

// DisjointSets is the Go port of the lock-free union-find in
// src/disjoint_sets.h (originally https://github.com/wjakob/dset,
// with connectedComponents added by the manifold project).
//
// The C++ version uses std::atomic<uint64_t> per cell to support
// parallel Unite, packing (rank << 32 | parent) into one 64-bit word.
// Our current Go callers are sequential, so we drop the atomic
// machinery and store the same (rank, parent) pair as plain uint32s.
// The algorithm semantics — union-by-rank with path compression and
// the same tie-break (lower id wins on equal rank) — are unchanged.
type DisjointSets struct {
	parent []uint32
	rank   []uint32
}

// NewDisjointSets builds a fresh union-find with `size` singleton
// sets (each element is its own parent, rank 0).
func NewDisjointSets(size int) *DisjointSets {
	d := &DisjointSets{
		parent: make([]uint32, size),
		rank:   make([]uint32, size),
	}
	for i := range d.parent {
		d.parent[i] = uint32(i)
	}
	return d
}

// Size returns the number of elements.
func (d *DisjointSets) Size() int { return len(d.parent) }

// Find returns the representative of the set containing id.
// Mirrors C++ findImpl with path compression.
func (d *DisjointSets) Find(id int) int {
	x := uint32(id)
	for d.parent[x] != x {
		// One-step path compression (point at grandparent).
		gp := d.parent[d.parent[x]]
		d.parent[x] = gp
		x = gp
	}
	return int(x)
}

// Unite merges the sets containing a and b using union-by-rank. Ties
// (equal rank) are broken by element id — the lower id becomes the
// new root, matching C++.
func (d *DisjointSets) Unite(a, b int) int {
	x := uint32(d.Find(a))
	y := uint32(d.Find(b))
	if x == y {
		return int(x)
	}
	rx, ry := d.rank[x], d.rank[y]
	// Swap so that y becomes the new root: C++ chooses the side with
	// larger rank, or (on equal rank) the larger id.
	if rx > ry || (rx == ry && x < y) {
		x, y = y, x
		rx, ry = ry, rx
	}
	d.parent[x] = y
	if rx == ry {
		d.rank[y] = ry + 1
	}
	return int(y)
}

// Same reports whether a and b are in the same set.
func (d *DisjointSets) Same(a, b int) bool {
	return d.Find(a) == d.Find(b)
}

// ConnectedComponents writes a component label per element into out
// and returns the total component count. Lonely nodes (rank 0 with
// no children) get fresh sequential labels; multi-element components
// share the same label per root id. Mirrors the C++ logic that
// optimizes the rank-0 path with no hashmap lookup.
func (d *DisjointSets) ConnectedComponents(out []int32) int {
	if cap(out) < len(d.parent) {
		out = append(out[:0], make([]int32, len(d.parent))...)
	}
	out = out[:len(d.parent)]
	lonely := 0
	toLabel := make(map[uint32]int32, len(d.parent))
	for i := range d.parent {
		root := uint32(d.Find(i))
		if d.rank[root] == 0 {
			out[i] = int32(len(toLabel)) + int32(lonely)
			lonely++
			continue
		}
		if lbl, ok := toLabel[root]; ok {
			out[i] = lbl
		} else {
			s := int32(len(toLabel)) + int32(lonely)
			toLabel[root] = s
			out[i] = s
		}
	}
	return len(toLabel) + lonely
}
