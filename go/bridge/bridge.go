// Package bridge is the cgo seam between Go and the C++ manifold library.
//
// It is throwaway scaffolding. Every symbol exported from this package
// represents a piece of the library not yet ported to pure Go. As inner
// layers get ported, calls into the corresponding C functions disappear and
// the bridge shrinks. The package is empty at the end of the port.
//
// Bridge functions currently route through the public manifoldc C API.
// When a port goes deeper than what manifoldc exposes, add a custom shim in
// bridge.cpp/bridge.h.
package bridge

// #cgo CFLAGS: -I${SRCDIR}/../../bindings/c/include
// #cgo CXXFLAGS: -std=c++17 -DMANIFOLD_CROSS_SECTION -DMANIFOLD_PAR=1 -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I${SRCDIR}/../../include -I${SRCDIR}/../../src -I${SRCDIR}/../../bindings/c -I${SRCDIR}/../../bindings/c/include -I${SRCDIR}/../../build/include -I${SRCDIR}/../../build/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem /opt/homebrew/include
// #cgo darwin LDFLAGS: -L${SRCDIR}/../../build/bindings/c -L${SRCDIR}/../../build/src -L/opt/homebrew/opt/tbb/lib -lmanifoldc -lmanifold -ltbb -Wl,-rpath,${SRCDIR}/../../build/bindings/c -Wl,-rpath,${SRCDIR}/../../build/src -Wl,-rpath,/opt/homebrew/opt/tbb/lib
// #cgo linux LDFLAGS: -L${SRCDIR}/../../build/bindings/c -L${SRCDIR}/../../build/src -lmanifoldc -lmanifold -ltbb -Wl,-rpath,${SRCDIR}/../../build/bindings/c -Wl,-rpath,${SRCDIR}/../../build/src
// #include <manifold/manifoldc.h>
// #include "bridge.h"
import "C"

import (
	"sync"
	"unsafe"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/handle"
)

// Impl is a Go-side handle to a C++ Manifold::Impl, retained via shared_ptr
// on the C side. Created by GetImpl, released by Delete. Methods on Impl
// expose chunks of the Impl's internal state to Go.
type Impl struct{ p *C.mb_impl_handle }

// GetImpl obtains the leaf impl backing a Manifold (lazy-evaluating any
// pending CSG ops). Pair with Delete.
func GetImpl(h *handle.Manifold) *Impl {
	return &Impl{p: C.mb_get_impl((*C.ManifoldManifold)(h.Ptr()))}
}

// Delete releases the Go-side hold on the C++ shared_ptr. Slices returned
// from Verts (and similar accessors) MUST NOT be used after Delete.
func (i *Impl) Delete() { C.mb_delete_impl(i.p) }

// Verts returns a Go slice aliasing the C++ Impl's vertPos_ array.
// Read-only; valid only while i has not been Deleted. The slice element
// layout (three packed float64s) matches the C++ vec<double,3> layout.
func (i *Impl) Verts() []geom.Vec3 {
	var n C.size_t
	p := C.mb_impl_vert_data(i.p, &n)
	if n == 0 {
		return nil
	}
	return unsafe.Slice((*geom.Vec3)(p), int(n))
}

// HalfedgeCount returns the count of halfedges in the Impl. NumTri,
// NumEdge, and IsEmpty all derive from this in Go.
func (i *Impl) HalfedgeCount() int { return int(C.mb_impl_halfedge_size(i.p)) }

// Minkowski calls C++ Impl::Minkowski(other, inset). Returns a Manifold
// directly (not an Impl) because Impl::Minkowski builds its result via
// the Boolean machinery, which produces a Manifold.
func (i *Impl) Minkowski(other *Impl, inset bool) *handle.Manifold {
	d := C.int(0)
	if inset {
		d = 1
	}
	return handle.NewManifold(unsafe.Pointer(C.mb_impl_minkowski(i.p, other.p, d)))
}

// MinGap calls C++ Impl::MinGap(other, searchLength).
func (i *Impl) MinGap(other *Impl, searchLength float64) float64 {
	return float64(C.mb_impl_min_gap(i.p, other.p, C.double(searchLength)))
}

// Copy returns a mutable copy of this Impl, suitable for ports that
// need to call non-const Impl methods (InitializeOriginal, etc.). The
// source is unaffected. Pair with MutableImpl.Delete.
func (i *Impl) Copy() *MutableImpl {
	return &MutableImpl{p: C.mb_impl_copy(i.p)}
}

// NewMutableImpl returns a fresh empty mutable Impl, mirroring
// `std::make_shared<Manifold::Impl>()`. Used by ports that need to fill
// in an Impl from scratch (e.g. Hull builds the hull mesh into a new Impl).
func NewMutableImpl() *MutableImpl {
	return &MutableImpl{p: C.mb_new_mutable_impl()}
}

// (bridge.Invalid removed — Invalid is now native Go in
// manifold.invalidManifold via MakeEmpty.)

// Verts returns a Go slice aliasing the mutable Impl's vertPos_ array.
// The slice is writable — modifying it modifies the C++ buffer
// directly. Same layout assumption as Impl.Verts.
func (mi *MutableImpl) Verts() []geom.Vec3 {
	var n C.size_t
	p := C.mb_mutable_impl_verts_data(mi.p, &n)
	if n == 0 {
		return nil
	}
	return unsafe.Slice((*geom.Vec3)(unsafe.Pointer(p)), int(n))
}

// SubdivideN wraps Impl::Subdivide with the constant-n splits-per-edge
// lambda (mirrors C++ Sphere's subdivision call).
func (mi *MutableImpl) SubdivideN(n int) {
	C.mb_mutable_impl_subdivide_n(mi.p, C.int(n))
}

// SetEpsilonMin wraps Impl::SetEpsilon(minEpsilon, false) — the
// one-arg overload used by ReadOBJ to restore epsilon from the file
// header.
func (mi *MutableImpl) SetEpsilonMin(minEpsilon float64) {
	C.mb_mutable_impl_set_epsilon_min(mi.p, C.double(minEpsilon))
}

// ImplReserveIDs wraps the static Manifold::Impl::ReserveIDs(n).
func ImplReserveIDs(n uint32) uint32 {
	return uint32(C.mb_impl_reserve_ids(C.uint(n)))
}

// CreateTangentsIdx calls Impl::CreateTangents(normalIdx) — fills
// halfedgeTangent_ from per-vert normals at the given property slot.
func (mi *MutableImpl) CreateTangentsIdx(normalIdx int) {
	C.mb_mutable_impl_create_tangents_idx(mi.p, C.int(normalIdx))
}

// CreateTangentsFromSmoothness calls Impl::CreateTangents with the
// given (halfedge, smoothness) pairs supplied as a Go slice. The C++
// side constructs a std::vector<Smoothness> internally.
func (mi *MutableImpl) CreateTangentsFromSmoothness(edges []Smoothness) {
	n := len(edges)
	if n == 0 {
		C.mb_mutable_impl_create_tangents_from_raw(mi.p, nil, nil, 0)
		return
	}
	halfedges := make([]C.size_t, n)
	smoothness := make([]C.double, n)
	for i, e := range edges {
		halfedges[i] = C.size_t(e.Halfedge)
		smoothness[i] = C.double(e.Smoothness)
	}
	C.mb_mutable_impl_create_tangents_from_raw(mi.p,
		(*C.size_t)(unsafe.Pointer(&halfedges[0])),
		(*C.double)(unsafe.Pointer(&smoothness[0])),
		C.size_t(n))
}

// SetNormals wraps Impl::SetNormals(normalIdx, minSharpAngle).
func (mi *MutableImpl) SetNormals(normalIdx int, minSharpAngle float64) {
	C.mb_mutable_impl_set_normals(mi.p,
		C.int(normalIdx), C.double(minSharpAngle))
}

// (PolygonsHandle / Impl.Slice / Impl.Project removed — Slice and
// Project are now native Go; see impl_slice.go and impl_project.go.)


// VertNormals returns a Go slice aliasing the Impl's vertNormal_ buffer.
// Read-only; valid only while i has not been Deleted. May be empty if
// the impl has no vertex normals computed.
func (i *Impl) VertNormals() []geom.Vec3 {
	var n C.size_t
	p := C.mb_impl_vert_normals_data(i.p, &n)
	if n == 0 {
		return nil
	}
	return unsafe.Slice((*geom.Vec3)(unsafe.Pointer(p)), int(n))
}

// VertNormals returns a writable view of vertNormal_ on the mutable impl.
func (mi *MutableImpl) VertNormals() []geom.Vec3 {
	var n C.size_t
	p := C.mb_mutable_impl_vert_normals_data(mi.p, &n)
	if n == 0 {
		return nil
	}
	return unsafe.Slice((*geom.Vec3)(unsafe.Pointer(p)), int(n))
}

// ResizeVerts wraps vertPos_.resize(n).
func (mi *MutableImpl) ResizeVerts(n int) {
	C.mb_mutable_impl_resize_verts(mi.p, C.size_t(n))
}

// ResizeVertNormals wraps vertNormal_.resize(n).
func (mi *MutableImpl) ResizeVertNormals(n int) {
	C.mb_mutable_impl_resize_vert_normals(mi.p, C.size_t(n))
}

// SetEpsilonValue assigns impl->epsilon_ directly (no SetEpsilon
// computation; just a field write).
func (mi *MutableImpl) SetEpsilonValue(epsilon float64) {
	C.mb_mutable_impl_set_epsilon_value(mi.p, C.double(epsilon))
}

// (bridge.DisjointSets removed — now implemented in pure Go; see
// disjoint_sets.go in the manifold package.)

// MeshGL64 and MeshGL handle types and per-field cgo accessors have
// been removed; GetMeshGL/GetMeshGL64 are now implemented natively in
// Go (see impl_meshgl.go) reading directly from the Impl's exposed
// buffers.

// Triangulate wraps the public free function manifold::Triangulate.
// Returns a flat []int32 of triangle indices (3 per triangle).
func Triangulate(polys [][]geom.Vec2, epsilon float64) []int32 {
	var totalPoints int
	for _, p := range polys {
		totalPoints += len(p)
	}
	flat := make([]geom.Vec2, 0, totalPoints)
	sizes := make([]C.size_t, len(polys))
	for i, p := range polys {
		flat = append(flat, p...)
		sizes[i] = C.size_t(len(p))
	}
	var fp unsafe.Pointer
	if totalPoints > 0 {
		fp = unsafe.Pointer(&flat[0])
	}
	var sp unsafe.Pointer
	if len(sizes) > 0 {
		sp = unsafe.Pointer(&sizes[0])
	}
	h := C.mb_triangulate(
		(*C.double)(fp), (*C.size_t)(sp), C.size_t(len(polys)),
		C.double(epsilon),
	)
	defer C.mb_delete_tri_verts(h)
	n := int(C.mb_tri_verts_count(h))
	if n == 0 {
		return nil
	}
	src := unsafe.Slice((*int32)(unsafe.Pointer(C.mb_tri_verts_data(h))), n*3)
	out := make([]int32, n*3)
	copy(out, src)
	return out
}

// SetHalfedgesRaw replaces the contents of halfedge_ with the three
// supplied parallel int32 arrays. All arrays must have the same length
// (== 3 * NumTri). Used by the Go port of CreateHalfedges to write
// the result of its sort+pair computation back into the C++ Impl.
func (mi *MutableImpl) SetHalfedgesRaw(starts, props, paireds []int32) {
	n := len(starts)
	if len(props) != n || len(paireds) != n {
		panic("bridge.SetHalfedgesRaw: arrays must have equal length")
	}
	if n == 0 {
		C.mb_mutable_impl_set_halfedges_raw(mi.p, nil, nil, nil, 0)
		return
	}
	C.mb_mutable_impl_set_halfedges_raw(mi.p,
		(*C.int)(unsafe.Pointer(&starts[0])),
		(*C.int)(unsafe.Pointer(&props[0])),
		(*C.int)(unsafe.Pointer(&paireds[0])),
		C.size_t(n))
}

// GetBBox returns the cached bBox_.min / bBox_.max corners. Used by
// ports that need to read the mesh extent (e.g. SetEpsilon's
// MaxEpsilon needs bBox_.Scale()).
func (mi *MutableImpl) GetBBox() (minV, maxV geom.Vec3) {
	var mnX, mnY, mnZ, mxX, mxY, mxZ C.double
	C.mb_mutable_impl_get_bbox(mi.p, &mnX, &mnY, &mnZ, &mxX, &mxY, &mxZ)
	return geom.Vec3{X: float64(mnX), Y: float64(mnY), Z: float64(mnZ)},
		geom.Vec3{X: float64(mxX), Y: float64(mxY), Z: float64(mxZ)}
}

// GetTolerance returns the current tolerance_ value. Used by ports
// that need to max-combine with a new minimum (e.g. SetEpsilon).
func (mi *MutableImpl) GetTolerance() float64 {
	return float64(C.mb_mutable_impl_get_tolerance(mi.p))
}

// SetBBox writes bBox_.min and bBox_.max directly. Used by the Go
// port of Impl::CalculateBBox to install the Go-computed corners.
func (mi *MutableImpl) SetBBox(min, max geom.Vec3) {
	C.mb_mutable_impl_set_bbox(mi.p,
		C.double(min.X), C.double(min.Y), C.double(min.Z),
		C.double(max.X), C.double(max.Y), C.double(max.Z))
}

// MakeEmpty wraps Impl::MakeEmpty(Error). status is the integer value
// of the C++ Manifold::Error enum (0 = NoError).
func (mi *MutableImpl) MakeEmpty(status int) {
	C.mb_mutable_impl_make_empty(mi.p, C.int(status))
}

// SetMeshRelationOriginalID assigns meshRelation_.originalID.
func (mi *MutableImpl) SetMeshRelationOriginalID(id int) {
	C.mb_mutable_impl_set_meshrelation_original_id(mi.p, C.int(id))
}

// SetTriRefs replaces the meshRelation_.triRef array with the four
// supplied parallel int32 arrays. All four must have the same length
// (== NumTri).
func (mi *MutableImpl) SetTriRefs(meshIDs, originalIDs, faceIDs, coplanarIDs []int32) {
	n := len(meshIDs)
	if len(originalIDs) != n || len(faceIDs) != n || len(coplanarIDs) != n {
		panic("bridge.SetTriRefs: arrays must have equal length")
	}
	if n == 0 {
		C.mb_mutable_impl_set_tri_refs(mi.p, nil, nil, nil, nil, 0)
		return
	}
	C.mb_mutable_impl_set_tri_refs(mi.p,
		(*C.int)(unsafe.Pointer(&meshIDs[0])),
		(*C.int)(unsafe.Pointer(&originalIDs[0])),
		(*C.int)(unsafe.Pointer(&faceIDs[0])),
		(*C.int)(unsafe.Pointer(&coplanarIDs[0])),
		C.size_t(n))
}

// ClearMeshIDTransforms removes all entries from
// meshRelation_.meshIDtransform.
func (mi *MutableImpl) ClearMeshIDTransforms() {
	C.mb_mutable_impl_clear_meshid_transforms(mi.p)
}

// ResizeFaceNormals resizes faceNormal_ to n elements. Pair with
// FaceNormalsMut for Go-side writes.
func (mi *MutableImpl) ResizeFaceNormals(n int) {
	C.mb_mutable_impl_resize_face_normals(mi.p, C.size_t(n))
}

// FaceNormalsMut returns a writable Go slice aliasing faceNormal_.
// Use ResizeFaceNormals first to size it correctly.
func (mi *MutableImpl) FaceNormalsMut() []geom.Vec3 {
	var n C.size_t
	p := C.mb_mutable_impl_face_normals_data(mi.p, &n)
	if n == 0 {
		return nil
	}
	return unsafe.Slice((*geom.Vec3)(unsafe.Pointer(p)), int(n))
}

// HalfedgeStartsRO returns a read-only Go slice aliasing halfedge_.start_
// on this MutableImpl. Read-only because the layout invariants are
// algorithm-specific; use the SetHalfedgesRaw bulk setter for writes.
// Valid only while this MutableImpl is alive.
func (mi *MutableImpl) HalfedgeStartsRO() []int32 {
	var n C.size_t
	p := C.mb_mutable_impl_halfedge_starts(mi.p, &n)
	if n == 0 {
		return nil
	}
	return unsafe.Slice((*int32)(unsafe.Pointer(p)), int(n))
}

// HalfedgePairsRO is the read-only paired-edge counterpart of
// HalfedgeStartsRO.
func (mi *MutableImpl) HalfedgePairsRO() []int32 {
	var n C.size_t
	p := C.mb_mutable_impl_halfedge_pairs(mi.p, &n)
	if n == 0 {
		return nil
	}
	return unsafe.Slice((*int32)(unsafe.Pointer(p)), int(n))
}

// HalfedgePropsRO is the read-only propVert_ counterpart of
// HalfedgeStartsRO.
func (mi *MutableImpl) HalfedgePropsRO() []int32 {
	var n C.size_t
	p := C.mb_mutable_impl_halfedge_props(mi.p, &n)
	if n == 0 {
		return nil
	}
	return unsafe.Slice((*int32)(unsafe.Pointer(p)), int(n))
}

// NumProp returns the impl's numProp_ (per-vertex property count).
// Mirrors Impl.Scalars().NumProp on the const handle.
func (mi *MutableImpl) NumProp() int {
	return int(C.mb_mutable_impl_num_prop(mi.p))
}

// Properties returns a read-only slice aliasing properties_ on a
// mutable Impl. Mirror of the const-Impl Properties accessor.
func (mi *MutableImpl) Properties() []float64 {
	var n C.size_t
	p := C.mb_mutable_impl_properties(mi.p, &n)
	if n == 0 {
		return nil
	}
	return unsafe.Slice((*float64)(unsafe.Pointer(p)), int(n))
}

// TriRefs reads meshRelation_.triRef from a mutable Impl, copying it
// into a Go slice. Mirror of the const-Impl TriRefs accessor.
func (mi *MutableImpl) TriRefs() []TriRef {
	var n C.size_t
	var meshIDs, originalIDs, faceIDs, coplanarIDs *C.int
	C.mb_mutable_impl_tri_refs(mi.p, &n, &meshIDs, &originalIDs, &faceIDs, &coplanarIDs)
	if n == 0 {
		return nil
	}
	out := make([]TriRef, int(n))
	for k := 0; k < int(n); k++ {
		base := uintptr(k) * unsafe.Sizeof(out[0])
		out[k].MeshID = int32(*(*C.int)(unsafe.Pointer(uintptr(unsafe.Pointer(meshIDs)) + base)))
		out[k].OriginalID = int32(*(*C.int)(unsafe.Pointer(uintptr(unsafe.Pointer(originalIDs)) + base)))
		out[k].FaceID = int32(*(*C.int)(unsafe.Pointer(uintptr(unsafe.Pointer(faceIDs)) + base)))
		out[k].CoplanarID = int32(*(*C.int)(unsafe.Pointer(uintptr(unsafe.Pointer(coplanarIDs)) + base)))
	}
	return out
}

// HalfedgeTangents returns a read-only slice aliasing halfedgeTangent_
// on a mutable Impl (4 packed doubles per element). Mirror of the
// const-Impl HalfedgeTangents accessor.
func (mi *MutableImpl) HalfedgeTangents() []float64 {
	var n C.size_t
	p := C.mb_mutable_impl_halfedge_tangents(mi.p, &n)
	if n == 0 {
		return nil
	}
	return unsafe.Slice((*float64)(unsafe.Pointer(p)), int(n)*4)
}

// SetNumProp writes numProp_. Caller must keep properties_ length in
// sync (numProp_ * NumPropVert) via SetProperties.
func (mi *MutableImpl) SetNumProp(n int) {
	C.mb_mutable_impl_set_num_prop(mi.p, C.int(n))
}

// SetProperties writes properties_ wholesale. Length must equal
// numProp_ * NumPropVert; caller is responsible for the consistency.
func (mi *MutableImpl) SetProperties(data []float64) {
	if len(data) == 0 {
		C.mb_mutable_impl_set_properties_raw(mi.p, nil, 0)
		return
	}
	C.mb_mutable_impl_set_properties_raw(mi.p,
		(*C.double)(unsafe.Pointer(&data[0])),
		C.size_t(len(data)))
}

// SetHalfedgeTangents writes halfedgeTangent_ wholesale. data is
// 4*n doubles laid out as packed vec4s. Pass empty to clear.
func (mi *MutableImpl) SetHalfedgeTangents(data []float64) {
	if len(data) == 0 {
		C.mb_mutable_impl_set_halfedge_tangents_raw(mi.p, nil, 0)
		return
	}
	n := len(data) / 4
	C.mb_mutable_impl_set_halfedge_tangents_raw(mi.p,
		(*C.double)(unsafe.Pointer(&data[0])),
		C.size_t(n))
}

// MeshIDTransforms reads meshRelation_.meshIDtransform from a mutable
// Impl in std::map order (ascending meshID). Mirror of the const-side
// Impl.MeshIDTransforms.
func (mi *MutableImpl) MeshIDTransforms() []MeshIDRelation {
	n := int(C.mb_mutable_impl_meshid_transform_count(mi.p))
	if n == 0 {
		return nil
	}
	meshIDs := make([]C.int, n)
	originalIDs := make([]C.int, n)
	transforms := make([]C.double, n*12)
	backSides := make([]C.uchar, n)
	hasNormals := make([]C.uchar, n)
	C.mb_mutable_impl_meshid_transforms(mi.p,
		(*C.int)(unsafe.Pointer(&meshIDs[0])),
		(*C.int)(unsafe.Pointer(&originalIDs[0])),
		(*C.double)(unsafe.Pointer(&transforms[0])),
		(*C.uchar)(unsafe.Pointer(&backSides[0])),
		(*C.uchar)(unsafe.Pointer(&hasNormals[0])))
	out := make([]MeshIDRelation, n)
	for k := 0; k < n; k++ {
		out[k].MeshID = int32(meshIDs[k])
		out[k].OriginalID = int32(originalIDs[k])
		for col := 0; col < 4; col++ {
			for row := 0; row < 3; row++ {
				out[k].Transform[col][row] = float64(transforms[k*12+col*3+row])
			}
		}
		out[k].BackSide = backSides[k] != 0
		out[k].HasNormals = hasNormals[k] != 0
	}
	return out
}

// SetCoplanarIDs writes only the coplanarID field of every TriRef in
// meshRelation_.triRef. Length of ids must equal NumTri (the triRef
// array must already be sized — typically by a prior SetTriRefs call
// or by Impl construction).
func (mi *MutableImpl) SetCoplanarIDs(ids []int32) {
	if len(ids) == 0 {
		C.mb_mutable_impl_set_coplanar_ids(mi.p, nil, 0)
		return
	}
	C.mb_mutable_impl_set_coplanar_ids(mi.p,
		(*C.int)(unsafe.Pointer(&ids[0])),
		C.size_t(len(ids)))
}

// AddMeshIDTransform inserts a single entry into
// meshRelation_.meshIDtransform.
//
// transform is a 3x4 column-major affine matrix (4 columns of 3
// doubles each; cols 0..2 linear, col 3 translation).
func (mi *MutableImpl) AddMeshIDTransform(meshID, originalID int,
	transform [4][3]float64, backSide, hasNormals bool) {
	bs := C.int(0)
	if backSide {
		bs = 1
	}
	hn := C.int(0)
	if hasNormals {
		hn = 1
	}
	C.mb_mutable_impl_add_meshid_transform(mi.p,
		C.int(meshID), C.int(originalID),
		C.double(transform[0][0]), C.double(transform[0][1]), C.double(transform[0][2]),
		C.double(transform[1][0]), C.double(transform[1][1]), C.double(transform[1][2]),
		C.double(transform[2][0]), C.double(transform[2][1]), C.double(transform[2][2]),
		C.double(transform[3][0]), C.double(transform[3][1]), C.double(transform[3][2]),
		bs, hn)
}

// warpRegistry tracks Go callback functions across a cgo call into
// Impl::Warp. Each call gets a fresh ID that the C++ side passes back
// through mbWarpTrampoline (declared via //export below).
var warpRegistry = struct {
	sync.Mutex
	next uintptr
	fns  map[uintptr]func(*geom.Vec3)
}{fns: map[uintptr]func(*geom.Vec3){}}

func registerWarpFn(fn func(*geom.Vec3)) uintptr {
	warpRegistry.Lock()
	defer warpRegistry.Unlock()
	warpRegistry.next++
	id := warpRegistry.next
	warpRegistry.fns[id] = fn
	return id
}

func unregisterWarpFn(id uintptr) {
	warpRegistry.Lock()
	defer warpRegistry.Unlock()
	delete(warpRegistry.fns, id)
}

//export mbWarpTrampoline
func mbWarpTrampoline(id C.uintptr_t, xyz *C.double) {
	warpRegistry.Lock()
	fn := warpRegistry.fns[uintptr(id)]
	warpRegistry.Unlock()
	if fn == nil {
		return
	}
	arr := unsafe.Slice((*float64)(unsafe.Pointer(xyz)), 3)
	v := &geom.Vec3{X: arr[0], Y: arr[1], Z: arr[2]}
	fn(v)
	arr[0] = v.X
	arr[1] = v.Y
	arr[2] = v.Z
}

// Warp calls Impl::Warp with the Go-supplied per-vertex function. The
// callback receives a pointer to a Vec3 and may mutate it in place to
// change that vertex's position.
func (mi *MutableImpl) Warp(fn func(*geom.Vec3)) {
	id := registerWarpFn(fn)
	defer unregisterWarpFn(id)
	C.mb_mutable_impl_warp(mi.p, C.uintptr_t(id))
}

// warpBatchRegistry tracks Go callbacks for Impl::WarpBatch.
var warpBatchRegistry = struct {
	sync.Mutex
	next uintptr
	fns  map[uintptr]func([]geom.Vec3)
}{fns: map[uintptr]func([]geom.Vec3){}}

func registerWarpBatchFn(fn func([]geom.Vec3)) uintptr {
	warpBatchRegistry.Lock()
	defer warpBatchRegistry.Unlock()
	warpBatchRegistry.next++
	id := warpBatchRegistry.next
	warpBatchRegistry.fns[id] = fn
	return id
}

func unregisterWarpBatchFn(id uintptr) {
	warpBatchRegistry.Lock()
	defer warpBatchRegistry.Unlock()
	delete(warpBatchRegistry.fns, id)
}

//export mbWarpBatchTrampoline
func mbWarpBatchTrampoline(id C.uintptr_t, p *C.double, n C.size_t) {
	warpBatchRegistry.Lock()
	fn := warpBatchRegistry.fns[uintptr(id)]
	warpBatchRegistry.Unlock()
	if fn == nil || n == 0 {
		return
	}
	verts := unsafe.Slice((*geom.Vec3)(unsafe.Pointer(p)), int(n))
	fn(verts)
}

// WarpBatch calls Impl::WarpBatch with the Go-supplied function. The
// callback receives the entire vertex slice aliasing C++ memory and
// may mutate in place.
func (mi *MutableImpl) WarpBatch(fn func([]geom.Vec3)) {
	id := registerWarpBatchFn(fn)
	defer unregisterWarpBatchFn(id)
	C.mb_mutable_impl_warp_batch(mi.p, C.uintptr_t(id))
}

// ManifoldFromMeshGL64 wraps the C++ Manifold(MeshGL64) constructor.
// All optional fields may be nil; only NumProp/vertProperties/triVerts
// and tolerance are required.
func ManifoldFromMeshGL64(
	numProp int,
	vertProperties []float64,
	triVerts []uint64,
	mergeFromVert []uint64,
	mergeToVert []uint64,
	runIndex []uint64,
	runOriginalID []uint32,
	runTransform []float64,
	runFlags []uint8,
	faceID []uint64,
	halfedgeTangent []float64,
	tolerance float64,
) *handle.Manifold {
	ptrF64 := func(s []float64) (*C.double, C.size_t) {
		if len(s) == 0 {
			return nil, 0
		}
		return (*C.double)(unsafe.Pointer(&s[0])), C.size_t(len(s))
	}
	ptrU64 := func(s []uint64) (*C.uint64_t, C.size_t) {
		if len(s) == 0 {
			return nil, 0
		}
		return (*C.uint64_t)(unsafe.Pointer(&s[0])), C.size_t(len(s))
	}
	ptrU32 := func(s []uint32) (*C.uint32_t, C.size_t) {
		if len(s) == 0 {
			return nil, 0
		}
		return (*C.uint32_t)(unsafe.Pointer(&s[0])), C.size_t(len(s))
	}
	ptrU8 := func(s []uint8) (*C.uint8_t, C.size_t) {
		if len(s) == 0 {
			return nil, 0
		}
		return (*C.uint8_t)(unsafe.Pointer(&s[0])), C.size_t(len(s))
	}
	vpPtr, vpLen := ptrF64(vertProperties)
	tvPtr, tvLen := ptrU64(triVerts)
	mfPtr, mfLen := ptrU64(mergeFromVert)
	mtPtr, mtLen := ptrU64(mergeToVert)
	riPtr, riLen := ptrU64(runIndex)
	roPtr, roLen := ptrU32(runOriginalID)
	rtPtr, rtLen := ptrF64(runTransform)
	rfPtr, rfLen := ptrU8(runFlags)
	fiPtr, fiLen := ptrU64(faceID)
	htPtr, htLen := ptrF64(halfedgeTangent)
	p := C.mb_manifold_from_meshgl64(
		C.size_t(numProp),
		vpPtr, vpLen,
		tvPtr, tvLen,
		mfPtr, mfLen,
		mtPtr, mtLen,
		riPtr, riLen,
		roPtr, roLen,
		rtPtr, rtLen,
		rfPtr, rfLen,
		fiPtr, fiLen,
		htPtr, htLen,
		C.double(tolerance),
	)
	return handle.NewManifold(unsafe.Pointer(p))
}

// ManifoldFromMeshGL wraps the C++ Manifold(MeshGL) constructor
// (float/uint32 variant).
func ManifoldFromMeshGL(
	numProp int,
	vertProperties []float32,
	triVerts []uint32,
	mergeFromVert []uint32,
	mergeToVert []uint32,
	runIndex []uint32,
	runOriginalID []uint32,
	runTransform []float32,
	runFlags []uint8,
	faceID []uint32,
	halfedgeTangent []float32,
	tolerance float32,
) *handle.Manifold {
	ptrF32 := func(s []float32) (*C.float, C.size_t) {
		if len(s) == 0 {
			return nil, 0
		}
		return (*C.float)(unsafe.Pointer(&s[0])), C.size_t(len(s))
	}
	ptrU32 := func(s []uint32) (*C.uint32_t, C.size_t) {
		if len(s) == 0 {
			return nil, 0
		}
		return (*C.uint32_t)(unsafe.Pointer(&s[0])), C.size_t(len(s))
	}
	ptrU8 := func(s []uint8) (*C.uint8_t, C.size_t) {
		if len(s) == 0 {
			return nil, 0
		}
		return (*C.uint8_t)(unsafe.Pointer(&s[0])), C.size_t(len(s))
	}
	vpPtr, vpLen := ptrF32(vertProperties)
	tvPtr, tvLen := ptrU32(triVerts)
	mfPtr, mfLen := ptrU32(mergeFromVert)
	mtPtr, mtLen := ptrU32(mergeToVert)
	riPtr, riLen := ptrU32(runIndex)
	roPtr, roLen := ptrU32(runOriginalID)
	rtPtr, rtLen := ptrF32(runTransform)
	rfPtr, rfLen := ptrU8(runFlags)
	fiPtr, fiLen := ptrU32(faceID)
	htPtr, htLen := ptrF32(halfedgeTangent)
	p := C.mb_manifold_from_meshgl(
		C.size_t(numProp),
		vpPtr, vpLen,
		tvPtr, tvLen,
		mfPtr, mfLen,
		mtPtr, mtLen,
		riPtr, riLen,
		roPtr, roLen,
		rtPtr, rtLen,
		rfPtr, rfLen,
		fiPtr, fiLen,
		htPtr, htLen,
		C.float(tolerance),
	)
	return handle.NewManifold(unsafe.Pointer(p))
}

// Smoothness mirrors manifold::Smoothness (include/manifold/common.h).
// The halfedge index = 3*tri + i, paired with a smoothness factor in
// [0, 1] (0 = sharp crease, 1 = fully smooth).
type Smoothness struct {
	Halfedge   uint64
	Smoothness float64
}

// SmoothFromMeshGL64 wraps the static Manifold::Smooth(MeshGL64,
// sharpenedEdges). Mirrors ManifoldFromMeshGL64 plus the sharpened
// edges array. All optional mesh fields may be nil.
func SmoothFromMeshGL64(
	numProp int,
	vertProperties []float64,
	triVerts []uint64,
	mergeFromVert []uint64,
	mergeToVert []uint64,
	runIndex []uint64,
	runOriginalID []uint32,
	runTransform []float64,
	runFlags []uint8,
	faceID []uint64,
	halfedgeTangent []float64,
	tolerance float64,
	sharpenedEdges []Smoothness,
) *handle.Manifold {
	ptrF64 := func(s []float64) (*C.double, C.size_t) {
		if len(s) == 0 {
			return nil, 0
		}
		return (*C.double)(unsafe.Pointer(&s[0])), C.size_t(len(s))
	}
	ptrU64 := func(s []uint64) (*C.uint64_t, C.size_t) {
		if len(s) == 0 {
			return nil, 0
		}
		return (*C.uint64_t)(unsafe.Pointer(&s[0])), C.size_t(len(s))
	}
	ptrU32 := func(s []uint32) (*C.uint32_t, C.size_t) {
		if len(s) == 0 {
			return nil, 0
		}
		return (*C.uint32_t)(unsafe.Pointer(&s[0])), C.size_t(len(s))
	}
	ptrU8 := func(s []uint8) (*C.uint8_t, C.size_t) {
		if len(s) == 0 {
			return nil, 0
		}
		return (*C.uint8_t)(unsafe.Pointer(&s[0])), C.size_t(len(s))
	}
	vpPtr, vpLen := ptrF64(vertProperties)
	tvPtr, tvLen := ptrU64(triVerts)
	mfPtr, mfLen := ptrU64(mergeFromVert)
	mtPtr, mtLen := ptrU64(mergeToVert)
	riPtr, riLen := ptrU64(runIndex)
	roPtr, roLen := ptrU32(runOriginalID)
	rtPtr, rtLen := ptrF64(runTransform)
	rfPtr, rfLen := ptrU8(runFlags)
	fiPtr, fiLen := ptrU64(faceID)
	htPtr, htLen := ptrF64(halfedgeTangent)

	// Split the Smoothness pairs into two parallel arrays for the C ABI.
	halfedges := make([]C.size_t, len(sharpenedEdges))
	smooths := make([]C.double, len(sharpenedEdges))
	for i, s := range sharpenedEdges {
		halfedges[i] = C.size_t(s.Halfedge)
		smooths[i] = C.double(s.Smoothness)
	}
	var shePtr *C.size_t
	var shsPtr *C.double
	if len(sharpenedEdges) > 0 {
		shePtr = (*C.size_t)(unsafe.Pointer(&halfedges[0]))
		shsPtr = (*C.double)(unsafe.Pointer(&smooths[0]))
	}

	p := C.mb_manifold_smooth_meshgl64(
		C.size_t(numProp),
		vpPtr, vpLen,
		tvPtr, tvLen,
		mfPtr, mfLen,
		mtPtr, mtLen,
		riPtr, riLen,
		roPtr, roLen,
		rtPtr, rtLen,
		rfPtr, rfLen,
		fiPtr, fiLen,
		htPtr, htLen,
		C.double(tolerance),
		shePtr, shsPtr, C.size_t(len(sharpenedEdges)),
	)
	return handle.NewManifold(unsafe.Pointer(p))
}

// SmoothFromMeshGL wraps the static Manifold::Smooth(MeshGL,
// sharpenedEdges) — float/uint32 variant.
func SmoothFromMeshGL(
	numProp int,
	vertProperties []float32,
	triVerts []uint32,
	mergeFromVert []uint32,
	mergeToVert []uint32,
	runIndex []uint32,
	runOriginalID []uint32,
	runTransform []float32,
	runFlags []uint8,
	faceID []uint32,
	halfedgeTangent []float32,
	tolerance float32,
	sharpenedEdges []Smoothness,
) *handle.Manifold {
	ptrF32 := func(s []float32) (*C.float, C.size_t) {
		if len(s) == 0 {
			return nil, 0
		}
		return (*C.float)(unsafe.Pointer(&s[0])), C.size_t(len(s))
	}
	ptrU32 := func(s []uint32) (*C.uint32_t, C.size_t) {
		if len(s) == 0 {
			return nil, 0
		}
		return (*C.uint32_t)(unsafe.Pointer(&s[0])), C.size_t(len(s))
	}
	ptrU8 := func(s []uint8) (*C.uint8_t, C.size_t) {
		if len(s) == 0 {
			return nil, 0
		}
		return (*C.uint8_t)(unsafe.Pointer(&s[0])), C.size_t(len(s))
	}
	vpPtr, vpLen := ptrF32(vertProperties)
	tvPtr, tvLen := ptrU32(triVerts)
	mfPtr, mfLen := ptrU32(mergeFromVert)
	mtPtr, mtLen := ptrU32(mergeToVert)
	riPtr, riLen := ptrU32(runIndex)
	roPtr, roLen := ptrU32(runOriginalID)
	rtPtr, rtLen := ptrF32(runTransform)
	rfPtr, rfLen := ptrU8(runFlags)
	fiPtr, fiLen := ptrU32(faceID)
	htPtr, htLen := ptrF32(halfedgeTangent)

	halfedges := make([]C.size_t, len(sharpenedEdges))
	smooths := make([]C.double, len(sharpenedEdges))
	for i, s := range sharpenedEdges {
		halfedges[i] = C.size_t(s.Halfedge)
		smooths[i] = C.double(s.Smoothness)
	}
	var shePtr *C.size_t
	var shsPtr *C.double
	if len(sharpenedEdges) > 0 {
		shePtr = (*C.size_t)(unsafe.Pointer(&halfedges[0]))
		shsPtr = (*C.double)(unsafe.Pointer(&smooths[0]))
	}

	p := C.mb_manifold_smooth_meshgl(
		C.size_t(numProp),
		vpPtr, vpLen,
		tvPtr, tvLen,
		mfPtr, mfLen,
		mtPtr, mtLen,
		riPtr, riLen,
		roPtr, roLen,
		rtPtr, rtLen,
		rfPtr, rfLen,
		fiPtr, fiLen,
		htPtr, htLen,
		C.float(tolerance),
		shePtr, shsPtr, C.size_t(len(sharpenedEdges)),
	)
	return handle.NewManifold(unsafe.Pointer(p))
}

// MatchesTriNormals wraps Impl::MatchesTriNormals() — returns true when
// every face normal agrees with the cross product of its triangle's
// halfedges (a validity check used by tests).
func (i *Impl) MatchesTriNormals() bool {
	return C.mb_impl_matches_tri_normals(i.p) != 0
}

// NumDegenerateTris wraps Impl::NumDegenerateTris() — number of
// triangles with zero or near-zero area.
func (i *Impl) NumDegenerateTris() int {
	return int(C.mb_impl_num_degenerate_tris(i.p))
}

// Boolean3 wraps the C++ Boolean3 helper (src/boolean3.h). Used by
// Split: a single Boolean3 evaluation can yield both the Intersect and
// the Subtract result, which is cheaper than running two booleans.
// Pair with Delete.
type Boolean3 struct{ p *C.mb_boolean3_handle }

// NewBoolean3 mirrors `Boolean3 boolean(*a, *b, op);` — the op (using
// the same integer values as the C++ OpType enum: 0=Add, 1=Subtract,
// 2=Intersect) passed at construction selects the
// intersection-extraction strategy used internally; Result(op) can then
// be called for any of the three op types.
func NewBoolean3(a, b *Impl, op int) *Boolean3 {
	return &Boolean3{p: C.mb_boolean3_new(a.p, b.p, C.int(op))}
}

// Result(op) calls Boolean3::Result(op) and returns the resulting
// Manifold (wrapped in a fresh ManifoldManifold handle).
func (b *Boolean3) Result(op int) *handle.Manifold {
	return handle.NewManifold(unsafe.Pointer(C.mb_boolean3_result_as_manifold(b.p, C.int(op))))
}

func (b *Boolean3) Delete() { C.mb_delete_boolean3(b.p) }

// RayHit is the Go mirror of C++ struct RayHit / C ManifoldRayHit.
type RayHit struct {
	FaceID   uint64
	Distance float64
	Position geom.Vec3
	Normal   geom.Vec3
}

// RayCast wraps Impl::RayCast(origin, endpoint). Returns the hits in
// Go-owned form; no handle to release.
func (i *Impl) RayCast(origin, endpoint geom.Vec3) []RayHit {
	vec := C.mb_impl_ray_cast(i.p,
		C.double(origin.X), C.double(origin.Y), C.double(origin.Z),
		C.double(endpoint.X), C.double(endpoint.Y), C.double(endpoint.Z))
	defer C.manifold_delete_ray_hit_vec(vec)
	n := int(C.manifold_ray_hit_vec_length(vec))
	out := make([]RayHit, n)
	for j := 0; j < n; j++ {
		h := C.manifold_ray_hit_vec_get(vec, C.size_t(j))
		out[j] = RayHit{
			FaceID:   uint64(h.face_id),
			Distance: float64(h.distance),
			Position: geom.Vec3{X: float64(h.position.x), Y: float64(h.position.y), Z: float64(h.position.z)},
			Normal:   geom.Vec3{X: float64(h.normal.x), Y: float64(h.normal.y), Z: float64(h.normal.z)},
		}
	}
	return out
}

// (MeshGLHandle / MeshGL handle types removed — see comment above the
// MeshGL64Handle deletion. GetMeshGL is now native Go in
// impl_meshgl.go.)

// (bridge.Extrude removed — Manifold::Extrude is now native Go;
// see impl_extrude.go.)

// MutableImpl wraps a non-const shared_ptr<Manifold::Impl>. Returned by
// Impl.Copy. Mutator methods on MutableImpl mirror the non-const
// methods on C++ Manifold::Impl.
type MutableImpl struct{ p *C.mb_mutable_impl_handle }

// SimplifyTopology calls C++ Impl::SimplifyTopology.
func (mi *MutableImpl) SimplifyTopology() {
	C.mb_mutable_impl_simplify_topology(mi.p)
}

// BuildCollider builds collider_ from the supplied (boxes, morton)
// arrays, refreshes bBox_ from the new collider, and runs
// CompactProps. boxes is flat 6 doubles per face (min then max).
// Used by the Go-side post-vert sort pipeline.
func (mi *MutableImpl) BuildCollider(boxes []float64, morton []uint32) {
	n := len(morton)
	if n == 0 {
		C.mb_mutable_impl_build_collider(mi.p, nil, nil, 0)
		return
	}
	C.mb_mutable_impl_build_collider(mi.p,
		(*C.double)(unsafe.Pointer(&boxes[0])),
		(*C.uint32_t)(unsafe.Pointer(&morton[0])),
		C.size_t(n))
}

// SetToleranceValue mirrors the C++ direct field assignment
// `impl->tolerance_ = tol`. It does not run the higher-level
// SetTolerance public API (which also calls SetNormalsAndCoplanar etc.).
func (mi *MutableImpl) SetToleranceValue(tol float64) {
	C.mb_mutable_impl_set_tolerance_value(mi.p, C.double(tol))
}

// RefineN calls Impl::Refine with the constant n-1 splits-per-edge
// lambda from C++ Manifold::Refine.
func (mi *MutableImpl) RefineN(n int) {
	C.mb_mutable_impl_refine_n(mi.p, C.int(n))
}

// RefineToLength calls Impl::Refine with the |edge|/length lambda from
// C++ Manifold::RefineToLength.
func (mi *MutableImpl) RefineToLength(length float64) {
	C.mb_mutable_impl_refine_to_length(mi.p, C.double(length))
}

// RefineToTolerance calls Impl::Refine with the tangent-aware lambda
// from C++ Manifold::RefineToTolerance. Caller must check that the Impl
// actually has halfedgeTangents before calling (matches C++ behavior).
func (mi *MutableImpl) RefineToTolerance(tolerance float64) {
	C.mb_mutable_impl_refine_to_tolerance(mi.p, C.double(tolerance))
}

// ToManifold wraps the mutable Impl in a Manifold via FromImpl. The Impl
// continues to be reference-counted by both this MutableImpl and the
// resulting Manifold; either may be deleted independently.
func (mi *MutableImpl) ToManifold() *handle.Manifold {
	return handle.NewManifold(unsafe.Pointer(C.mb_manifold_from_mutable_impl(mi.p)))
}

// Delete releases the Go-side hold on the C++ shared_ptr<Impl>.
func (mi *MutableImpl) Delete() { C.mb_delete_mutable_impl(mi.p) }

// (bridge.PropagateStatus removed — propagateStatus is now native
// Go in the manifold package via MakeEmpty.)

// CsgNode wraps a C++ shared_ptr<CsgNode>. Produced by LoadPNode (mirror
// of Manifold::LoadPNode) and CsgNode.Transform (mirror of
// CsgNode::Transform). Pair every CsgNode with Delete.
type CsgNode struct{ p *C.mb_csg_node_handle }

// LoadPNode wraps the private Manifold::LoadPNode, which returns the
// underlying CsgNode tree without forcing leaf evaluation.
func LoadPNode(h *handle.Manifold) *CsgNode {
	return &CsgNode{p: C.mb_manifold_load_pnode((*C.ManifoldManifold)(h.Ptr()))}
}

// Transform calls C++ CsgNode::Transform with the 12-element 3x4
// matrix (column-major), returning a new CsgNode that represents the
// deferred transform.
func (n *CsgNode) Transform(
	x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4 float64,
) *CsgNode {
	return &CsgNode{p: C.mb_csg_node_transform(
		n.p,
		C.double(x1), C.double(y1), C.double(z1),
		C.double(x2), C.double(y2), C.double(z2),
		C.double(x3), C.double(y3), C.double(z3),
		C.double(x4), C.double(y4), C.double(z4),
	)}
}

// Boolean calls C++ CsgNode::Boolean(other, op), returning a new
// CsgNode (typically a CsgOpNode) representing the deferred operation.
// op uses the same integer values as the C++ OpType enum:
// 0 = Add (Union), 1 = Subtract (Difference), 2 = Intersect.
func (n *CsgNode) Boolean(other *CsgNode, op int) *CsgNode {
	return &CsgNode{p: C.mb_csg_node_boolean(n.p, other.p, C.int(op))}
}

// NewCsgOpNode wraps `make_shared<CsgOpNode>(children, op)` — combines
// many CsgNode children under a single op (e.g. union of N manifolds).
// Used by BatchBoolean / Compose.
func NewCsgOpNode(nodes []*CsgNode, op int) *CsgNode {
	if len(nodes) == 0 {
		return nil
	}
	cnodes := make([]*C.mb_csg_node_handle, len(nodes))
	for i, n := range nodes {
		cnodes[i] = n.p
	}
	return &CsgNode{p: C.mb_csg_op_node(
		(**C.mb_csg_node_handle)(unsafe.Pointer(&cnodes[0])),
		C.size_t(len(nodes)),
		C.int(op),
	)}
}

// ToManifold wraps this CsgNode in a Manifold via the private
// Manifold(shared_ptr<CsgNode>) constructor (accessed through the
// ManifoldBridge friend).
func (n *CsgNode) ToManifold() *handle.Manifold {
	return handle.NewManifold(unsafe.Pointer(C.mb_manifold_from_csg_node(n.p)))
}

// Delete releases the Go-side hold on the shared_ptr<CsgNode>.
func (n *CsgNode) Delete() { C.mb_delete_csg_node(n.p) }

// HalfedgeStarts returns a Go slice aliasing the start-vert array of the
// Impl's halfedges (the SoA start_ buffer of class Halfedges). Triangle t
// has vertices at HalfedgeStarts()[3t], [3t+1], [3t+2]. Read-only; valid
// only while i has not been Deleted.
// HalfedgePairs returns a Go slice aliasing the paired-edge array of
// halfedge_. Element value -1 marks a boundary halfedge; otherwise it
// is the index of the paired halfedge. Read-only; valid only while
// this Impl is alive.
// HalfedgeProps returns a read-only slice aliasing halfedge_.propVert_
// on a const Impl. Layout: int32[3*NumTri].
func (i *Impl) HalfedgeProps() []int32 {
	var n C.size_t
	p := C.mb_impl_halfedge_props(i.p, &n)
	if n == 0 {
		return nil
	}
	return unsafe.Slice((*int32)(unsafe.Pointer(p)), int(n))
}

// HalfedgeTangents returns a read-only slice aliasing halfedgeTangent_
// on a const Impl. Layout: 4 packed doubles per element (x, y, z, w);
// the returned slice contains 4*NumHalfedge doubles.
func (i *Impl) HalfedgeTangents() []float64 {
	var n C.size_t
	p := C.mb_impl_halfedge_tangents(i.p, &n)
	if n == 0 {
		return nil
	}
	return unsafe.Slice((*float64)(unsafe.Pointer(p)), int(n)*4)
}

// Properties returns a read-only slice aliasing properties_ on a const
// Impl. Length = NumProp * NumPropVert; entries are interleaved
// numProp doubles per propVert.
func (i *Impl) Properties() []float64 {
	var n C.size_t
	p := C.mb_impl_properties(i.p, &n)
	if n == 0 {
		return nil
	}
	return unsafe.Slice((*float64)(unsafe.Pointer(p)), int(n))
}

// TriRef is the Go mirror of C++ TriRef.
type TriRef struct {
	MeshID, OriginalID, FaceID, CoplanarID int32
}

// TriRefs reads the meshRelation_.triRef array from a const Impl,
// copying it into a Go slice. The four interleaved int fields in
// each C++ TriRef map to one Go TriRef struct per triangle.
func (i *Impl) TriRefs() []TriRef {
	var n C.size_t
	var meshIDs, originalIDs, faceIDs, coplanarIDs *C.int
	C.mb_impl_tri_refs(i.p, &n, &meshIDs, &originalIDs, &faceIDs, &coplanarIDs)
	if n == 0 {
		return nil
	}
	// Each pointer points to one field within the first TriRef element.
	// Stride between elements is sizeof(TriRef) = 4 ints. Read via the
	// strided pointer arithmetic that unsafe.Slice doesn't directly
	// support; copy field-by-field instead.
	out := make([]TriRef, int(n))
	for k := 0; k < int(n); k++ {
		base := uintptr(k) * unsafe.Sizeof(out[0])
		out[k].MeshID = int32(*(*C.int)(unsafe.Pointer(uintptr(unsafe.Pointer(meshIDs)) + base)))
		out[k].OriginalID = int32(*(*C.int)(unsafe.Pointer(uintptr(unsafe.Pointer(originalIDs)) + base)))
		out[k].FaceID = int32(*(*C.int)(unsafe.Pointer(uintptr(unsafe.Pointer(faceIDs)) + base)))
		out[k].CoplanarID = int32(*(*C.int)(unsafe.Pointer(uintptr(unsafe.Pointer(coplanarIDs)) + base)))
	}
	return out
}

// MeshIDRelation is the Go mirror of one entry in
// meshRelation_.meshIDtransform.
type MeshIDRelation struct {
	MeshID, OriginalID  int32
	Transform           [4][3]float64
	BackSide, HasNormals bool
}

// MeshIDTransforms reads meshRelation_.meshIDtransform on a const Impl,
// returning all entries in std::map order (ascending meshID).
func (i *Impl) MeshIDTransforms() []MeshIDRelation {
	n := int(C.mb_impl_meshid_transform_count(i.p))
	if n == 0 {
		return nil
	}
	meshIDs := make([]C.int, n)
	originalIDs := make([]C.int, n)
	transforms := make([]C.double, n*12)
	backSides := make([]C.uchar, n)
	hasNormals := make([]C.uchar, n)
	C.mb_impl_meshid_transforms(i.p,
		(*C.int)(unsafe.Pointer(&meshIDs[0])),
		(*C.int)(unsafe.Pointer(&originalIDs[0])),
		(*C.double)(unsafe.Pointer(&transforms[0])),
		(*C.uchar)(unsafe.Pointer(&backSides[0])),
		(*C.uchar)(unsafe.Pointer(&hasNormals[0])))
	out := make([]MeshIDRelation, n)
	for k := 0; k < n; k++ {
		out[k].MeshID = int32(meshIDs[k])
		out[k].OriginalID = int32(originalIDs[k])
		for col := 0; col < 4; col++ {
			for row := 0; row < 3; row++ {
				out[k].Transform[col][row] = float64(transforms[k*12+col*3+row])
			}
		}
		out[k].BackSide = backSides[k] != 0
		out[k].HasNormals = hasNormals[k] != 0
	}
	return out
}

// BBox reads the cached bBox_ corners from a const Impl.
func (i *Impl) BBox() (minV, maxV geom.Vec3) {
	var mnX, mnY, mnZ, mxX, mxY, mxZ C.double
	C.mb_impl_bbox(i.p, &mnX, &mnY, &mnZ, &mxX, &mxY, &mxZ)
	return geom.Vec3{X: float64(mnX), Y: float64(mnY), Z: float64(mnZ)},
		geom.Vec3{X: float64(mxX), Y: float64(mxY), Z: float64(mxZ)}
}

func (i *Impl) HalfedgePairs() []int32 {
	var n C.size_t
	p := C.mb_impl_halfedge_pairs(i.p, &n)
	if n == 0 {
		return nil
	}
	return unsafe.Slice((*int32)(unsafe.Pointer(p)), int(n))
}

// FaceNormals returns a Go slice aliasing faceNormal_ (one vec3 per
// triangle). Empty when the impl has no cached face normals. Read-only;
// valid only while this Impl is alive.
func (i *Impl) FaceNormals() []geom.Vec3 {
	var n C.size_t
	p := C.mb_impl_face_normals(i.p, &n)
	if n == 0 {
		return nil
	}
	return unsafe.Slice((*geom.Vec3)(unsafe.Pointer(p)), int(n))
}

func (i *Impl) HalfedgeStarts() []int32 {
	var n C.size_t
	p := C.mb_impl_halfedge_starts(i.p, &n)
	if n == 0 {
		return nil
	}
	return unsafe.Slice((*int32)(p), int(n))
}

// ImplScalars is a Go-side view of small scalar fields on Manifold::Impl.
type ImplScalars struct {
	NumProp             int
	PropertiesSize      int
	HalfedgeTangentSize int
	Tolerance           float64
	Epsilon             float64
	OriginalID          int
	Status              int
}

// Scalars reads all the small scalar fields off the Impl in one cgo call.
// Used by accessor ports that previously each made their own bridge call.
func (i *Impl) Scalars() ImplScalars {
	var s C.mb_impl_scalars
	C.mb_impl_get_scalars(i.p, &s)
	return ImplScalars{
		NumProp:             int(s.num_prop),
		PropertiesSize:      int(s.properties_size),
		HalfedgeTangentSize: int(s.halfedge_tangent_size),
		Tolerance:           float64(s.tolerance),
		Epsilon:             float64(s.epsilon),
		OriginalID:          int(s.original_id),
		Status:              int(s.status),
	}
}

// Empty returns a handle to a new empty Manifold. The C++ Manifold default
// constructor's role; needed by ports that have an "invalid input → empty
// result" branch (e.g. Mirror with a zero normal).
func Empty() *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_empty(unsafe.Pointer(mem))
	return handle.NewManifold(unsafe.Pointer(p))
}


func DeleteManifold(h *handle.Manifold) {
	C.manifold_delete_manifold((*C.ManifoldManifold)(h.Ptr()))
}

// ExecutionContext wraps a C++ ExecutionContext. Used to observe
// progress and request cancellation of long-running evaluations.
// Pair construction with Delete.
type ExecutionContext struct{ p *C.ManifoldExecutionContext }

// NewExecutionContext mirrors `ExecutionContext ctx;` — constructs a
// fresh context whose cancel flag is clear and progress is 1.0 (no
// work scheduled yet).
func NewExecutionContext() *ExecutionContext {
	mem := C.manifold_alloc_execution_context()
	p := C.manifold_execution_context(unsafe.Pointer(mem))
	return &ExecutionContext{p: p}
}

// Cancel mirrors ExecutionContext::Cancel — idempotent, callable from
// any thread.
func (c *ExecutionContext) Cancel() {
	C.manifold_execution_context_cancel(c.p)
}

// Cancelled mirrors ExecutionContext::Cancelled.
func (c *ExecutionContext) Cancelled() bool {
	return C.manifold_execution_context_cancelled(c.p) != 0
}

// Progress mirrors ExecutionContext::Progress — normalized [0, 1]
// progress; 1.0 means no work scheduled or evaluation complete.
func (c *ExecutionContext) Progress() float64 {
	return float64(C.manifold_execution_context_progress(c.p))
}

// Delete frees the underlying C++ context. After Delete the Go value
// must not be used; Cancel/Progress on a deleted context is UB.
func (c *ExecutionContext) Delete() {
	C.manifold_delete_execution_context(c.p)
}

// WithContext mirrors Manifold::WithContext — returns a copy of m with
// ctx atomically attached. The next eager op on the returned Manifold
// observes ctx for progress and cancellation.
func WithContext(h *handle.Manifold, ctx *ExecutionContext) *handle.Manifold {
	mem := C.manifold_alloc_manifold()
	p := C.manifold_with_context(unsafe.Pointer(mem),
		(*C.ManifoldManifold)(h.Ptr()), ctx.p)
	return handle.NewManifold(unsafe.Pointer(p))
}

