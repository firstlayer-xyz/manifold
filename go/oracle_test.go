package manifold

import (
	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/handle"
)

// This file is the test-only seam between the native Go Manifold (which holds
// native storage, post oracle-migration step 3) and the C++ reference oracle
// (the bridge/reference packages, which speak in cgo handle.Manifold). Production
// no longer stores a bridge handle on a Manifold; tests that compare against the
// C++ oracle marshal across this seam.

// wrap takes ownership of a freshly-created bridge handle, marshals its Impl into
// native storage, and releases the handle. Used by tests that build a Manifold via
// a bridge constructor (e.g. bridge.ManifoldFromMeshGL64) and own the result.
func wrap(h *handle.Manifold) *Manifold {
	bi := bridge.GetImpl(h)
	m := &Manifold{s: marshalImplStorageFromBridge(bi)}
	bi.Delete()
	bridge.DeleteManifold(h)
	return m
}

// fromRefHandle marshals a reference/bridge handle's Impl into a native Manifold
// WITHOUT taking ownership of the handle — the caller retains it (and any later
// reference.DeleteManifold). Replaces the old `&Manifold{h: refHandle}` pattern.
func fromRefHandle(h *handle.Manifold) *Manifold {
	bi := bridge.GetImpl(h)
	defer bi.Delete()
	return &Manifold{s: marshalImplStorageFromBridge(bi)}
}

// refHandle turns this Manifold's native storage into a fresh, fully-valid C++
// handle for the reference oracle. It round-trips through the MeshGL ingest path:
// the C++ Manifold(MeshGL64) constructor re-finalizes (Morton-sorts the faces and
// builds collider_ from the sorted leaves), so the handle is valid for ANY native
// mesh — including transformed meshes whose faces are no longer Morton-sorted, on
// which a direct Collider rebuild (which assumes sorted input) would corrupt or
// crash. Replaces the old `m.h` field access. Each call allocates a new handle the
// caller is responsible for (reference.DeleteManifold).
func (m *Manifold) refHandle() *handle.Manifold {
	gl := m.GetMeshGL64(-1)
	return bridge.ManifoldFromMeshGL64(
		gl.NumProp,
		gl.VertProperties, gl.TriVerts,
		gl.MergeFromVert, gl.MergeToVert,
		gl.RunIndex, gl.RunOriginalID, gl.RunTransform, gl.RunFlags,
		gl.FaceID, gl.HalfedgeTangent,
		gl.Tolerance,
	)
}
