// Package pool provides a generic node recycler for the stdcpp containers. The
// node-based containers (the multiset AVL tree, and future hash/tree maps) allocate
// one small node per insert; under the heavy insert/erase churn of the geometry
// algorithms that dominates GC by object count. A FreeList recycles those nodes so a
// container's steady-state allocations track its peak size, not its total churn.
package pool

// FreeList recycles *T allocations. It is NOT safe for concurrent use — each owning
// container keeps its own FreeList (the manifold containers are used single-threaded
// within an operation). Put zeroes the element before pooling it, so the free pool
// never pins stale references (important for node types that hold child pointers);
// Get therefore always returns a zeroed *T, exactly like new(T).
type FreeList[T any] struct {
	free []*T
}

// Get returns a zeroed *T, reusing a recycled element when one is available.
func (f *FreeList[T]) Get() *T {
	n := len(f.free)
	if n == 0 {
		return new(T)
	}
	p := f.free[n-1]
	f.free[n-1] = nil // don't let the backing array pin the handed-out element
	f.free = f.free[:n-1]
	return p
}

// Put returns p to the pool after zeroing it. p must not be used afterwards.
func (f *FreeList[T]) Put(p *T) {
	var zero T
	*p = zero
	f.free = append(f.free, p)
}
