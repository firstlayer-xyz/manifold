package triangulate

// orderedMultiset is a balanced-BST (AVL) multiset that replicates
// std::multiset semantics for the ear-clipping triangulator: elements are
// ordered by a `less` comparator, and EQUAL elements are kept in insertion
// order (FIFO), so Begin() returns the earliest-inserted minimum. That FIFO
// tie-break among equal keys is exactly what makes the triangulation output
// match C++ byte-for-byte (libc++/libstdc++ multiset insert at upper_bound).
//
// Handles (msHandle) stay valid across other Insert/Erase: Erase locates the
// element by the (val, seq) total order rather than by node pointer, so the
// internal AVL value-copy delete never invalidates a handle.
//
// Contract (matches how std::multiset is used in EarClip): an element's key
// must not change while it is in the set; to re-key, Erase then Insert.
type orderedMultiset[V any] struct {
	root *msNode[V]
	less func(a, b V) bool
	seq  uint64
	size int
}

type msNode[V any] struct {
	val         V
	seq         uint64
	left, right *msNode[V]
	height      int
}

// msHandle identifies an inserted element by value + insertion sequence. The
// zero value (valid=false) is the "end" / "no element" sentinel.
type msHandle[V any] struct {
	val   V
	seq   uint64
	valid bool
}

// Value returns the element a handle refers to.
func (h msHandle[V]) Value() V { return h.val }

// Valid reports whether the handle refers to an element (vs the end sentinel).
func (h msHandle[V]) Valid() bool { return h.valid }

func newOrderedMultiset[V any](less func(a, b V) bool) *orderedMultiset[V] {
	return &orderedMultiset[V]{less: less}
}

// Len returns the number of elements.
func (m *orderedMultiset[V]) Len() int { return m.size }

// Clear empties the set. The insertion sequence keeps increasing (harmless;
// pre-Clear handles are never reused).
func (m *orderedMultiset[V]) Clear() {
	m.root = nil
	m.size = 0
}

// before is the total order: by `less`, then by insertion sequence (so equal
// keys order earliest-inserted first).
func (m *orderedMultiset[V]) before(av V, as uint64, bv V, bs uint64) bool {
	if m.less(av, bv) {
		return true
	}
	if m.less(bv, av) {
		return false
	}
	return as < bs
}

// Insert adds v and returns its stable handle.
func (m *orderedMultiset[V]) Insert(v V) msHandle[V] {
	m.seq++
	s := m.seq
	m.root = m.insert(m.root, v, s)
	m.size++
	return msHandle[V]{val: v, seq: s, valid: true}
}

func (m *orderedMultiset[V]) insert(n *msNode[V], v V, s uint64) *msNode[V] {
	if n == nil {
		return &msNode[V]{val: v, seq: s, height: 1}
	}
	if m.before(v, s, n.val, n.seq) {
		n.left = m.insert(n.left, v, s)
	} else {
		n.right = m.insert(n.right, v, s)
	}
	return msRebalance(n)
}

// Erase removes the element identified by h (a no-op for an invalid handle or
// an already-removed element).
func (m *orderedMultiset[V]) Erase(h msHandle[V]) {
	if !h.valid {
		return
	}
	var removed bool
	m.root, removed = m.deleteKey(m.root, h.val, h.seq)
	if removed {
		m.size--
	}
}

func (m *orderedMultiset[V]) deleteKey(n *msNode[V], v V, s uint64) (*msNode[V], bool) {
	if n == nil {
		return nil, false
	}
	var removed bool
	switch {
	case m.before(v, s, n.val, n.seq):
		n.left, removed = m.deleteKey(n.left, v, s)
	case m.before(n.val, n.seq, v, s):
		n.right, removed = m.deleteKey(n.right, v, s)
	default:
		// n is the target (unique by seq).
		removed = true
		if n.left == nil {
			return n.right, true
		}
		if n.right == nil {
			return n.left, true
		}
		// Two children: pull up the in-order successor, then delete it. The
		// (val, seq) total order keeps this BST-valid and handle-stable.
		succ := n.right
		for succ.left != nil {
			succ = succ.left
		}
		n.val, n.seq = succ.val, succ.seq
		n.right, _ = m.deleteKey(n.right, succ.val, succ.seq)
	}
	return msRebalance(n), removed
}

// Begin returns the minimum element's handle, and false if the set is empty.
func (m *orderedMultiset[V]) Begin() (msHandle[V], bool) {
	if m.root == nil {
		return msHandle[V]{}, false
	}
	n := m.root
	for n.left != nil {
		n = n.left
	}
	return msHandle[V]{val: n.val, seq: n.seq, valid: true}, true
}

// InOrder returns all elements in sorted order (used to iterate holes_).
func (m *orderedMultiset[V]) InOrder() []V {
	out := make([]V, 0, m.size)
	var rec func(n *msNode[V])
	rec = func(n *msNode[V]) {
		if n == nil {
			return
		}
		rec(n.left)
		out = append(out, n.val)
		rec(n.right)
	}
	rec(m.root)
	return out
}

// --- AVL helpers ---

func msHeight[V any](n *msNode[V]) int {
	if n == nil {
		return 0
	}
	return n.height
}

func msUpdate[V any](n *msNode[V]) {
	l, r := msHeight(n.left), msHeight(n.right)
	if l > r {
		n.height = l + 1
	} else {
		n.height = r + 1
	}
}

func msBalance[V any](n *msNode[V]) int { return msHeight(n.left) - msHeight(n.right) }

func msRotRight[V any](y *msNode[V]) *msNode[V] {
	x := y.left
	y.left = x.right
	x.right = y
	msUpdate(y)
	msUpdate(x)
	return x
}

func msRotLeft[V any](x *msNode[V]) *msNode[V] {
	y := x.right
	x.right = y.left
	y.left = x
	msUpdate(x)
	msUpdate(y)
	return y
}

func msRebalance[V any](n *msNode[V]) *msNode[V] {
	msUpdate(n)
	switch bf := msBalance(n); {
	case bf > 1:
		if msBalance(n.left) < 0 {
			n.left = msRotLeft(n.left)
		}
		return msRotRight(n)
	case bf < -1:
		if msBalance(n.right) > 0 {
			n.right = msRotRight(n.right)
		}
		return msRotLeft(n)
	default:
		return n
	}
}
