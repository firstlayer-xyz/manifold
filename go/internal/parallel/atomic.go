// AtomicAdd primitives mirroring src/utils.h::AtomicAdd. The C++
// version is a CAS loop over std::atomic<T>::compare_exchange_weak
// for floating-point T, and a fetch_add specialization for int.
// Go's sync/atomic exposes int32/int64/uint32/uint64 directly; for
// float64 we CAS over the bit pattern via math.Float64bits.

package parallel

import (
	"math"
	"sync/atomic"
	"unsafe"
)

// AtomicAddFloat64 is the Go port of C++ AtomicAdd<double> from
// src/utils.h. Performs *addr = *addr + delta atomically via a
// compare-and-swap loop on the bit pattern. Used by code that
// accumulates per-vertex contributions in parallel (e.g.
// CurvatureAngles) where multiple workers may touch the same slot.
func AtomicAddFloat64(addr *float64, delta float64) {
	p := (*uint64)(unsafe.Pointer(addr))
	for {
		old := atomic.LoadUint64(p)
		newBits := math.Float64bits(math.Float64frombits(old) + delta)
		if atomic.CompareAndSwapUint64(p, old, newBits) {
			return
		}
	}
}
