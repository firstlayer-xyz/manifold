// Package orderedmap is the Go replacement for C++ `std::map<K, V>`
// — a key-sorted associative container with O(1) lookup. C++
// `std::map` is a red-black tree with O(log n) on all ops; this
// port uses the simpler "sorted slice of keys + Go map of values"
// combination, which gives O(log n) on the probe + O(n) slice
// shift for insert/delete. That's fine because every use of
// `std::map` in the manifold codebase is on small N (≤ hundreds of
// entries, often < 10) — meshIDtransform, meshIDtoNormalTransform,
// hole2BBox_, etc.
//
// The key property this type preserves vs. a plain `map[K]V` is
// **sorted-key iteration**, matching `std::map`'s in-order
// traversal. This matters anywhere the iteration order affects
// the algorithm's output (e.g. ID assignment, deterministic
// processing).
package orderedmap

import (
	"cmp"
	"iter"
	"slices"
)

// OrderedMap mirrors `std::map<K, V>` semantics: sorted-key
// iteration, unique keys, O(1) value lookup.
type OrderedMap[K cmp.Ordered, V any] struct {
	keys []K
	vals map[K]V
}

// New returns an empty OrderedMap.
func New[K cmp.Ordered, V any]() *OrderedMap[K, V] {
	return &OrderedMap[K, V]{vals: make(map[K]V)}
}

// Len returns the number of entries.
func (m *OrderedMap[K, V]) Len() int { return len(m.keys) }

// Get returns the value associated with key (and true), or the
// zero value (and false) if key is absent. Mirrors C++
// `map::find` followed by check.
func (m *OrderedMap[K, V]) Get(key K) (V, bool) {
	v, ok := m.vals[key]
	return v, ok
}

// Set inserts or overwrites the value at key. Mirrors C++
// `map::operator[] = v` / `map::insert_or_assign`.
func (m *OrderedMap[K, V]) Set(key K, val V) {
	if _, ok := m.vals[key]; !ok {
		// New key — binary-insert into the sorted slice.
		i, _ := slices.BinarySearch(m.keys, key)
		m.keys = slices.Insert(m.keys, i, key)
	}
	m.vals[key] = val
}

// Delete removes the entry at key. Returns true iff the key was
// present. Mirrors C++ `map::erase(key)`.
func (m *OrderedMap[K, V]) Delete(key K) bool {
	if _, ok := m.vals[key]; !ok {
		return false
	}
	delete(m.vals, key)
	i, _ := slices.BinarySearch(m.keys, key)
	m.keys = slices.Delete(m.keys, i, i+1)
	return true
}

// Clear removes every entry.
func (m *OrderedMap[K, V]) Clear() {
	clear(m.vals)
	m.keys = m.keys[:0]
}

// Contains reports whether key is present.
func (m *OrderedMap[K, V]) Contains(key K) bool {
	_, ok := m.vals[key]
	return ok
}

// Keys returns a snapshot of the keys in sorted order. The returned
// slice aliases the internal storage and MUST NOT be mutated.
func (m *OrderedMap[K, V]) Keys() []K { return m.keys }

// All returns an iterator over (key, value) pairs in sorted-key
// order. Use with Go 1.23+ range-over-func:
//
//	for k, v := range m.All() {
//	    ...
//	}
func (m *OrderedMap[K, V]) All() iter.Seq2[K, V] {
	return func(yield func(K, V) bool) {
		for _, k := range m.keys {
			if !yield(k, m.vals[k]) {
				return
			}
		}
	}
}
