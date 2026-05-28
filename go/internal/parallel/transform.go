package parallel

// Transform is the Go port of manifold::transform from src/parallel.h.
// Writes out[i] = f(in[i]) for every i in [0, len(in)). Caller must
// ensure len(out) >= len(in).
//
// Implemented in terms of ForEachN — same parallel fan-out shape as
// the C++ tbb::parallel_for branch in transform().
func Transform[S any, T any](policy ExecutionPolicy, in []S, out []T, f func(s S) T) {
	n := len(in)
	ForEachN(policy, n, func(i int) {
		out[i] = f(in[i])
	})
}

// Fill is the Go port of manifold::fill from src/parallel.h. Writes
// value into every slot of slice.
func Fill[T any](policy ExecutionPolicy, slice []T, value T) {
	ForEachN(policy, len(slice), func(i int) {
		slice[i] = value
	})
}

// Sequence is the Go port of manifold::sequence from src/parallel.h.
// Writes slice[i] = T(i) for every i, mirroring std::iota's parallel
// equivalent used by the manifold code (always with int element type).
func Sequence(policy ExecutionPolicy, slice []int32) {
	ForEachN(policy, len(slice), func(i int) {
		slice[i] = int32(i)
	})
}

// Gather is the Go port of manifold::gather from src/parallel.h.
// out[i] = src[indices[i]] for every i. Caller ensures len(out) >=
// len(indices) and every indices[i] is in range for src.
func Gather[T any](policy ExecutionPolicy, indices []int32, src []T, out []T) {
	ForEachN(policy, len(indices), func(i int) {
		out[i] = src[indices[i]]
	})
}

// Scatter is the Go port of manifold::scatter from src/parallel.h.
// out[indices[i]] = src[i] for every i. As in C++, the caller is
// responsible for ensuring no duplicate destination indices; the
// parallel branch performs the writes concurrently without
// synchronization, identical to tbb::parallel_for.
func Scatter[T any](policy ExecutionPolicy, src []T, indices []int32, out []T) {
	ForEachN(policy, len(src), func(i int) {
		out[indices[i]] = src[i]
	})
}

// Permute is the Go port of manifold::Permute from src/utils.h.
// Reorders inOut so that result[i] = oldInOut[new2Old[i]]. The
// resulting slice has length len(new2Old) — which may differ from
// len(inOut), the C++ version uses Vec::resize_nofill to grow/shrink.
//
// Returns a fresh slice (the input slice is not aliased) to match
// the C++ "move into tmp, gather back into resized inOut" pattern.
func Permute[T any](policy ExecutionPolicy, inOut []T, new2Old []int32) []T {
	out := make([]T, len(new2Old))
	Gather(policy, new2Old, inOut, out)
	return out
}
