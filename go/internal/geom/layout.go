package geom

import "unsafe"

// Compile-time assertions that the Go-side primitive types match the C++
// memory layout that the bridge package aliases via unsafe.Slice. If these
// fail, every cgo buffer view (Verts, HalfedgeStarts, ...) silently
// corrupts. A non-zero array length here is a compile error.
const (
	_ uintptr = 24 - unsafe.Sizeof(Vec3{}) // C++ vec<double,3> = 3 doubles
	_ uintptr = 8 - unsafe.Alignof(Vec3{}) // 8-byte aligned (float64)
)
