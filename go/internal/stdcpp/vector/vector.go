// Package vector is a Go reimplementation of C++ std::vector<T> that reflects the C++
// type's PERFORMANCE, not just its semantics: a std::vector reuses its backing storage
// (and can be given a pool allocator), which in Go terms means recycling the backing
// array so short-lived scratch vectors reuse memory instead of churning the GC. The
// native port's GC cost comes almost entirely from such per-call scratch slices; the
// bare []T+append mirrors push_back faithfully in FORM but not in cost, and Pool/Vector
// close that gap without diverging (a std::vector with a pool allocator is real C++).
package vector

import "sync"

// Pool is a recycling allocator for []T backing arrays — the moral equivalent of a C++
// std::vector pool allocator. Get hands out a zero-length slice with reusable capacity
// (or nil to be grown fresh); Put returns a finished slice for reuse. Unlike sync.Pool
// it does NOT box (sync.Pool.Put boxes the value, trading a buffer allocation for a
// holder allocation — no net reduction in allocation COUNT, which is what drives Go GC).
// The mutex is uncontended for the single-threaded-per-operation manifold algorithms but
// keeps a single package-level Pool safe across concurrent operations.
type Pool[T any] struct {
	mu   sync.Mutex
	free [][]T
}

// Get returns a recycled []T (length 0, capacity reused) or nil when the pool is empty
// (the caller grows it with append, then returns it via Put).
func (p *Pool[T]) Get() []T {
	p.mu.Lock()
	n := len(p.free)
	if n == 0 {
		p.mu.Unlock()
		return nil
	}
	s := p.free[n-1]
	p.free[n-1] = nil // don't pin the handed-out buffer
	p.free = p.free[:n-1]
	p.mu.Unlock()
	return s[:0]
}

// Put returns s to the pool for reuse. The caller must not use s afterwards. A
// zero-capacity slice has no backing array to recycle and is dropped.
func (p *Pool[T]) Put(s []T) {
	if cap(s) == 0 {
		return
	}
	p.mu.Lock()
	p.free = append(p.free, s)
	p.mu.Unlock()
}

// Vector is a push_back-faithful std::vector<T> whose storage is drawn from and returned
// to a Pool. Build it with New(pool), append with PushBack, borrow the storage for
// interop with []T APIs via Slice, and return the storage with Release at end of life.
type Vector[T any] struct {
	s    []T
	pool *Pool[T]
}

// New returns an empty Vector backed by pool (its storage recycled from / to the pool).
func New[T any](pool *Pool[T]) Vector[T] {
	return Vector[T]{s: pool.Get(), pool: pool}
}

// PushBack appends x (std::vector::push_back; geometric growth via Go's append).
func (v *Vector[T]) PushBack(x T) { v.s = append(v.s, x) }

// Len is std::vector::size.
func (v *Vector[T]) Len() int { return len(v.s) }

// At is std::vector::operator[] (read).
func (v *Vector[T]) At(i int) T { return v.s[i] }

// Slice borrows the contiguous storage for interop with []T APIs. Valid until the next
// PushBack that grows past capacity, or Release.
func (v *Vector[T]) Slice() []T { return v.s }

// Clear empties the vector but keeps its capacity (std::vector::clear).
func (v *Vector[T]) Clear() { v.s = v.s[:0] }

// Release returns the backing storage to the pool. The Vector must not be used after.
func (v *Vector[T]) Release() {
	v.pool.Put(v.s)
	v.s = nil
}
