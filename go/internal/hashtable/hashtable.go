// Package hashtable is the Go port of src/hashtable.h — a lock-free
// open-addressing hashtable with linear probing, designed for
// concurrent insert under TBB / goroutine fan-out. Insert is
// race-free via compare-and-swap on the keys array; reads are
// non-blocking via atomic-load.
//
// The original C++ template surface is split into two types:
//   - HashTable[V] holds the backing arrays + the used counter.
//   - HashTableD[V] is a "view" — same operations, cheaper to
//     copy and pass into worker bodies. C++ uses the view as the
//     `D()` snapshot.
//
// The Go port collapses both into a single *HashTable[V] which is
// safe to share by pointer across goroutines (the backing arrays
// don't move; the used counter is sync/atomic-protected).
//
// Hash function: hash64bit from src/utils.h, baked in as Hash64Bit
// here — matches the canonical 64-bit avalanche from
// https://stackoverflow.com/questions/664014.
package hashtable

import (
	"math"
	"math/bits"
	"sync/atomic"
)

// Open is the sentinel "empty slot" key — matches C++ kOpen
// (numeric_limits<uint64_t>::max()).
const Open uint64 = math.MaxUint64

// Hash64Bit is the Go port of C++ hash64bit (src/utils.h). 64-bit
// avalanche derived from Murmur3 finalization. Constexpr in C++; we
// just declare it as a plain function.
func Hash64Bit(x uint64) uint64 {
	x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9
	x = (x ^ (x >> 27)) * 0x94d049bb133111eb
	x = x ^ (x >> 31)
	return x
}

// HashTable is a concurrent open-addressing hashtable keyed by
// uint64 with V-typed values. Mirrors C++ `HashTable<V, H>`.
type HashTable[V any] struct {
	keys   []uint64
	values []V
	used   atomic.Uint64
	step   uint32
}

// New creates a HashTable sized to the next power of two above
// `size`, with linear-probing step `step` (default 1).
//
// Step is a tuning knob (matches C++): step=1 is plain linear
// probing; larger odd values may reduce clustering at high load
// factors but require odd-vs-power-of-two coprimality which is
// automatic for any odd step.
func New[V any](size int, step uint32) *HashTable[V] {
	if size == 0 {
		return &HashTable[V]{step: step}
	}
	if step == 0 {
		step = 1
	}
	capacity := 1 << ceilLog2(uint64(size))
	h := &HashTable[V]{
		keys:   make([]uint64, capacity),
		values: make([]V, capacity),
		step:   step,
	}
	for i := range h.keys {
		h.keys[i] = Open
	}
	return h
}

// ceilLog2 mirrors C++ CeilLog2 (src/hashtable.h:56).
func ceilLog2(v uint64) int {
	if v <= 1 {
		return 0
	}
	return 64 - bits.LeadingZeros64(v-1)
}

// Size returns the capacity of the keys/values arrays. C++ HashTableD::Size.
func (h *HashTable[V]) Size() int { return len(h.keys) }

// Entries returns the number of occupied slots. C++ HashTable::Entries.
func (h *HashTable[V]) Entries() int { return int(h.used.Load()) }

// Full reports whether the table has crossed 50% load factor.
// Inserts past Full() are silently dropped — matches C++.
func (h *HashTable[V]) Full() bool {
	return int(h.used.Load())*2 > h.Size()
}

// FilledFraction returns used / capacity.
func (h *HashTable[V]) FilledFraction() float64 {
	if h.Size() == 0 {
		return 0
	}
	return float64(h.used.Load()) / float64(h.Size())
}

// Insert is the Go port of C++ HashTableD::Insert. Atomic CAS on
// the keys array claims a slot; the value is written non-atomically
// once the key is locked. Linear probe with step `step`.
//
// Returns silently if the table is full or the key already exists.
// Matches C++ semantics exactly.
func (h *HashTable[V]) Insert(key uint64, val V) {
	if h.Size() == 0 {
		return
	}
	idx := uint32(Hash64Bit(key)) & uint32(h.Size()-1)
	for {
		if h.Full() {
			return
		}
		// Atomic CAS — mirrors C++ AtomicCAS at src/hashtable.h:64-68.
		// Go's CompareAndSwapUint64 returns success bool; we read
		// the actual value if it failed.
		if atomic.CompareAndSwapUint64(&h.keys[idx], Open, key) {
			h.used.Add(1)
			h.values[idx] = val
			return
		}
		// CAS failed — reload to see what's there.
		k := atomic.LoadUint64(&h.keys[idx])
		if k == key {
			return
		}
		idx = (idx + h.step) & uint32(h.Size()-1)
	}
}

// Lookup is the Go port of C++ HashTableD::operator[](key). Returns
// the value at the slot whose key matches (or the zero value when
// not found, with bool=false).
//
// NOTE: copies the value. If the value type has fields written
// concurrently by other goroutines (e.g. LevelSet's GridVert
// edgeVerts during ComputeVerts), use Slot instead and read only
// the specific fields the caller actually needs via the returned
// pointer.
func (h *HashTable[V]) Lookup(key uint64) (V, bool) {
	var zero V
	if h.Size() == 0 {
		return zero, false
	}
	idx := uint32(Hash64Bit(key)) & uint32(h.Size()-1)
	for {
		k := atomic.LoadUint64(&h.keys[idx])
		if k == key {
			return h.values[idx], true
		}
		if k == Open {
			return zero, false
		}
		idx = (idx + h.step) & uint32(h.Size()-1)
	}
}

// Slot is a race-aware variant of Lookup — returns a pointer to the
// value at the slot whose key matches, or (nil, false) when not
// found. Mirrors C++ HashTableD::operator[] which also returns a
// reference into values_.
//
// The caller MUST read fields off the returned *V using atomic
// loads (or with knowledge that the field's writes have already
// happened-before via outer synchronization). The plain Lookup
// alternative COPIES the entire V, which under -race trips on any
// field written concurrently — even if the caller doesn't intend
// to use that field.
func (h *HashTable[V]) Slot(key uint64) (*V, bool) {
	if h.Size() == 0 {
		return nil, false
	}
	idx := uint32(Hash64Bit(key)) & uint32(h.Size()-1)
	for {
		k := atomic.LoadUint64(&h.keys[idx])
		if k == key {
			return &h.values[idx], true
		}
		if k == Open {
			return nil, false
		}
		idx = (idx + h.step) & uint32(h.Size()-1)
	}
}

// KeyAt returns the key stored at slot idx (Open if unoccupied).
// Mirrors C++ HashTableD::KeyAt.
func (h *HashTable[V]) KeyAt(idx int) uint64 {
	return atomic.LoadUint64(&h.keys[idx])
}

// At returns a pointer to the value at slot idx. Mirrors C++
// HashTableD::At returning a V&. The pointer is stable for the
// lifetime of the HashTable (no resize) and may be written
// in-place by the caller — used by the LevelSet ComputeVerts pass
// which mutates the GridVert at each slot.
func (h *HashTable[V]) At(idx int) *V {
	return &h.values[idx]
}
