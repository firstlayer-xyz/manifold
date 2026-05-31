// Package multimap is the Go reimplementation of C++ std::multimap<K, V> for the
// operations the manifold port uses (AssembleHalfedges, face_op.cpp:44): multiple
// values per key, key-ordered, with FIFO order among equal keys. C++ multimap::find
// on a duplicate key returns an implementation-defined element of the equal range
// (libstdc++: the first); the FIFO front replicates that.
package multimap

import "cmp"

// Multimap mirrors std::multimap<K, V>: multiple values per key, ordered by key,
// FIFO among equal keys (insertion order). Only the operations the port uses are
// provided. Backed by a Go map of value slices + an element count.
type Multimap[K cmp.Ordered, V any] struct {
	vals map[K][]V
	n    int
}

// New returns an empty Multimap.
func New[K cmp.Ordered, V any]() *Multimap[K, V] {
	return &Multimap[K, V]{vals: map[K][]V{}}
}

// Emplace appends val under key (FIFO among equal keys). Mirrors multimap::emplace.
func (m *Multimap[K, V]) Emplace(key K, val V) {
	m.vals[key] = append(m.vals[key], val)
	m.n++
}

// Empty reports whether the multimap has no elements.
func (m *Multimap[K, V]) Empty() bool { return m.n == 0 }

// BeginValue is multimap::begin()->second: the first value of the smallest key.
// The caller must ensure the multimap is non-empty (matches C++ begin() on a
// non-empty container).
func (m *Multimap[K, V]) BeginValue() V {
	var minKey K
	first := true
	for k, vs := range m.vals {
		if len(vs) == 0 {
			continue
		}
		if first || k < minKey {
			minKey, first = k, false
		}
	}
	return m.vals[minKey][0]
}

// FindFirst is multimap::find: the first value for key (ok=false if absent).
func (m *Multimap[K, V]) FindFirst(key K) (V, bool) {
	vs := m.vals[key]
	if len(vs) == 0 {
		var zero V
		return zero, false
	}
	return vs[0], true
}

// EraseFirst erases the find() result: the first value for key.
func (m *Multimap[K, V]) EraseFirst(key K) {
	m.vals[key] = m.vals[key][1:]
	m.n--
}
