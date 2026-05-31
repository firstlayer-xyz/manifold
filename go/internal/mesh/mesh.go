// Package mesh holds the native Go ports of the C++ mesh-topology and provenance
// types from src/shared.h and src/impl.h: Halfedge, Halfedges, TriRef, Relation,
// MeshRelationD. These are the storage types Manifold::Impl owns. Keeping them in
// one low-level package lets both the manifold facade and the internal/boolean
// engine share a single canonical definition (no import cycle: mesh imports only
// geom + orderedmap).
package mesh

import (
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/orderedmap"
)

// NextHalfedge is the Go port of NextHalfedge (shared.h:33): the next halfedge
// within the same triangle.
func NextHalfedge(current int) int { return current - current%3 + geom.Next3(current%3) }

// Halfedge is the Go port of the C++ Halfedge value struct (shared.h:174): a
// directed half-edge's start/end verts, paired halfedge, and property vert.
type Halfedge struct {
	StartVert, EndVert, PairedHalfedge, PropVert int
}

// IsForward mirrors Halfedge::IsForward (shared.h:178).
func (h Halfedge) IsForward() bool { return h.StartVert < h.EndVert }

// Less mirrors Halfedge::operator< (shared.h:179): startVert, then endVert.
func (h Halfedge) Less(o Halfedge) bool {
	if h.StartVert == o.StartVert {
		return h.EndVert < o.EndVert
	}
	return h.StartVert < o.StartVert
}

// Halfedges is the Go port of the C++ Halfedges class (shared.h:185): the mesh's
// halfedge structure-of-arrays (start_/paired_/propVert_). End is DERIVED from the
// next halfedge's start (not stored), so there is no SetEnd backing array.
type Halfedges struct {
	starts   []int32
	paired   []int32
	propVert []int32
}

// NewHalfedges wraps three parallel arrays (e.g. from bridge marshalling). The
// slices must be equal length; they are adopted, not copied.
func NewHalfedges(starts, paired, propVert []int32) Halfedges {
	return Halfedges{starts: starts, paired: paired, propVert: propVert}
}

// Raw exposes the backing arrays (start, paired, propVert) for marshalling.
func (h *Halfedges) Raw() (starts, paired, propVert []int32) {
	return h.starts, h.paired, h.propVert
}

// Starts / Paired / PropVert expose the individual backing arrays (read-write
// aliases) — the SoA columns halfedge_.start_ / .paired_ / .propVert_.
func (h *Halfedges) Starts() []int32   { return h.starts }
func (h *Halfedges) Paired() []int32   { return h.paired }
func (h *Halfedges) PropVert() []int32 { return h.propVert }

func (h *Halfedges) Size() int   { return len(h.starts) }
func (h *Halfedges) Empty() bool { return len(h.starts) == 0 }

func (h *Halfedges) Start(idx int) int { return int(h.starts[idx]) }
func (h *Halfedges) End(idx int) int   { return int(h.starts[NextHalfedge(idx)]) }
func (h *Halfedges) Pair(idx int) int  { return int(h.paired[idx]) }
func (h *Halfedges) Prop(idx int) int  { return int(h.propVert[idx]) }

func (h *Halfedges) SetStart(idx, vert int) { h.starts[idx] = int32(vert) }
func (h *Halfedges) SetEnd(idx, vert int)   { h.starts[NextHalfedge(idx)] = int32(vert) }
func (h *Halfedges) SetPair(idx, pair int)  { h.paired[idx] = int32(pair) }
func (h *Halfedges) SetProp(idx, prop int)  { h.propVert[idx] = int32(prop) }

// IsForward mirrors Halfedges::IsForward (shared.h:204).
func (h *Halfedges) IsForward(idx int) bool { return h.Start(idx) < h.End(idx) }

// Get mirrors Halfedges::Get (shared.h:206): a whole-halfedge snapshot.
func (h *Halfedges) Get(idx int) Halfedge {
	return Halfedge{StartVert: h.Start(idx), EndVert: h.End(idx), PairedHalfedge: h.Pair(idx), PropVert: h.Prop(idx)}
}

// Set mirrors Halfedges::Set (shared.h:210): start/pair/prop (End is derived).
func (h *Halfedges) Set(idx, startVert, pairedHalfedge, propVert int) {
	h.SetStart(idx, startVert)
	h.SetPair(idx, pairedHalfedge)
	h.SetProp(idx, propVert)
}

// PushBack mirrors Halfedges::push_back (shared.h:216).
func (h *Halfedges) PushBack(startVert, pairedHalfedge, propVert int) {
	h.starts = append(h.starts, int32(startVert))
	h.paired = append(h.paired, int32(pairedHalfedge))
	h.propVert = append(h.propVert, int32(propVert))
}

// Resize mirrors Halfedges::resize (shared.h:222): grow/shrink, new slots = -1.
func (h *Halfedges) Resize(n int) {
	h.starts = resizeFill(h.starts, n, -1)
	h.paired = resizeFill(h.paired, n, -1)
	h.propVert = resizeFill(h.propVert, n, -1)
}

// ResizeNoFill mirrors Halfedges::resize_nofill (shared.h:228): grow/shrink with
// new slots left zero (Go's zero value; the caller fills them).
func (h *Halfedges) ResizeNoFill(n int) {
	h.starts = resizeNoFill(h.starts, n)
	h.paired = resizeNoFill(h.paired, n)
	h.propVert = resizeNoFill(h.propVert, n)
}

func resizeFill(s []int32, n int, fill int32) []int32 {
	if n <= len(s) {
		return s[:n]
	}
	for len(s) < n {
		s = append(s, fill)
	}
	return s
}

func resizeNoFill(s []int32, n int) []int32 {
	if n <= len(s) {
		return s[:n]
	}
	return append(s, make([]int32, n-len(s))...)
}

// Barycentric is the Go port of the C++ Barycentric (shared.h:283): a new vert's
// originating triangle plus its barycentric (uvw, or quad uvwx) coordinates.
type Barycentric struct {
	Tri int
	UVW geom.Vec4
}

// TmpEdge is the Go port of the C++ TmpEdge (shared.h:315): a forward-only edge
// record referencing the halfedge it came from. First <= Second always.
type TmpEdge struct {
	First, Second, HalfedgeIdx int
}

// NewTmpEdge mirrors the C++ TmpEdge(start, end, idx) ctor: First/Second are the
// min/max of start/end.
func NewTmpEdge(start, end, idx int) TmpEdge {
	first, second := start, end
	if end < start {
		first, second = end, start
	}
	return TmpEdge{First: first, Second: second, HalfedgeIdx: idx}
}

// Less mirrors TmpEdge::operator< (shared.h:325): First, then Second.
func (e TmpEdge) Less(o TmpEdge) bool {
	if e.First == o.First {
		return e.Second < o.Second
	}
	return e.First < o.First
}

// CreateTmpEdges is the Go port of CreateTmpEdges (shared.h:330): one TmpEdge per
// forward halfedge, in halfedge-index order (the C++ builds one per halfedge then
// remove_if's the backward ones, which preserves the kept order).
func CreateTmpEdges(h *Halfedges) []TmpEdge {
	n := h.Size()
	edges := make([]TmpEdge, 0, n/2)
	for idx := 0; idx < n; idx++ {
		if !h.IsForward(idx) {
			continue
		}
		edges = append(edges, NewTmpEdge(h.Start(idx), h.End(idx), idx))
	}
	return edges
}

// TriRef is the Go port of the C++ TriRef (shared.h:288): the provenance of an
// output triangle — mesh-instance id, original mesh id, face id, coplanar id.
type TriRef struct {
	MeshID, OriginalID, FaceID, CoplanarID int
}

// SameFace mirrors TriRef::SameFace (shared.h:307): same mesh, coplanar, and face.
func (r TriRef) SameFace(o TriRef) bool {
	return r.MeshID == o.MeshID && r.CoplanarID == o.CoplanarID && r.FaceID == o.FaceID
}

// Relation is the Go port of the C++ Manifold::Impl::Relation (impl.h:29): one
// meshIDtransform entry. The transform defaults to identity (not the zero matrix).
type Relation struct {
	OriginalID int
	Transform  geom.Mat3x4
	BackSide   bool
	HasNormals bool
}

// identityMat3x4 is la::identity for a 3x4 affine (column-major: identity linear
// part + zero translation), the C++ default for Relation.transform.
var identityMat3x4 = geom.Mat3x4{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 0}}

// NewRelation returns a Relation with the C++ default transform (identity).
func NewRelation() Relation { return Relation{OriginalID: -1, Transform: identityMat3x4} }

// GetNormalTransform mirrors Relation::GetNormalTransform (impl.h:38).
func (r Relation) GetNormalTransform() geom.Mat3 {
	m := geom.NormalTransform(r.Transform)
	if r.BackSide {
		return m.MulScalar(-1.0)
	}
	return m
}

// GetInverseNormalTransform mirrors Relation::GetInverseNormalTransform (impl.h:42).
func (r Relation) GetInverseNormalTransform() geom.Mat3 {
	m := geom.InverseNormalTransform(r.Transform)
	if r.BackSide {
		return m.MulScalar(-1.0)
	}
	return m
}

// MeshRelationD is the Go port of the C++ Manifold::Impl::MeshRelationD (impl.h:46):
// the per-tri provenance (triRef), the meshID->Relation transform map (a std::map,
// i.e. ordered by key — orderedmap preserves that), and this mesh's originalID.
type MeshRelationD struct {
	OriginalID      int
	MeshIDTransform *orderedmap.OrderedMap[int, Relation]
	TriRef          []TriRef
}

// NewMeshRelationD returns an empty MeshRelationD (originalID -1, empty map).
func NewMeshRelationD() MeshRelationD {
	return MeshRelationD{OriginalID: -1, MeshIDTransform: orderedmap.New[int, Relation]()}
}
