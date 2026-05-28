// Package handle holds opaque pointer types shared between the public Go
// package and the cgo-using subpackages.
//
// It exists to break what would otherwise be an import cycle: the public
// manifold package needs to expose a wrapper type backed by a C pointer, and
// the bridge and reference subpackages need to operate on that same pointer
// without being able to import manifold. The handle type lives here so all
// three packages can share it.
package handle

import "unsafe"

// Manifold carries a raw pointer to a C++ ManifoldManifold owned by the
// caller. It does not own the underlying memory — finalizers and explicit
// deletion are the responsibility of whoever constructed it.
type Manifold struct {
	p unsafe.Pointer
}

func NewManifold(p unsafe.Pointer) *Manifold { return &Manifold{p: p} }

func (m *Manifold) Ptr() unsafe.Pointer { return m.p }
