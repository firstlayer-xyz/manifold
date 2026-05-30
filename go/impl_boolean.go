package manifold

import (
	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/boolean"
	"github.com/firstlayer-xyz/manifold/go/internal/collider"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// boolOperand builds the boolean.Operand the native Boolean3 reads from a const
// Impl: the kernel arrays (aliasing C++ memory — the Impl must outlive the
// Boolean3), the bbox, a fresh face collider, and epsilon/tolerance.
func boolOperand(v *Impl) boolean.Operand {
	min, max := v.BBox()
	fb, fm := v.GetFaceBoxMorton()
	s := v.Scalars()
	return boolean.Operand{
		VertPos:    v.Verts(),
		VertNormal: v.VertNormals(),
		FaceNormal: v.FaceNormals(),
		Starts:     v.HalfedgeStarts(),
		Pairs:      v.HalfedgePairs(),
		PropVert:   v.HalfedgeProps(),
		BBox:       geom.Box{Min: min, Max: max},
		Collider:   collider.New(fb, fm),
		Epsilon:    s.Epsilon,
		Tolerance:  s.Tolerance,
	}
}

// nativeBoolean3 is the native counterpart of bridge.Boolean3: it holds the
// computed boolean.Boolean3 plus its two input Impls (the C++ Boolean3 holds
// const Impl& inP_/inQ_), so Result(op) can build the output Impl and run the
// finalize tail. A single nativeBoolean3 can serve two Result calls (Split).
type nativeBoolean3 struct {
	b3       *boolean.Boolean3
	inP, inQ *Impl
}

// newNativeBoolean3 is the Go port of the Boolean3 ctor dispatch: expandP =
// (op == Add). The Operands alias the input Impls' memory, so inP/inQ must
// outlive the returned object.
func newNativeBoolean3(inP, inQ *Impl, op OpType) *nativeBoolean3 {
	b3 := boolean.NewBoolean3(boolOperand(inP), boolOperand(inQ), op == OpAdd)
	return &nativeBoolean3{b3: b3, inP: inP, inQ: inQ}
}

// Result is the Go port of Boolean3::Result (boolean_result.cpp:693): the native
// boolean.Result produces the triangulated mesh (assemble -> Face2Tri ->
// ReorderHalfedges); this builds the output Impl from it and runs the finalize
// tail (CreateProperties, UpdateReference, SimplifyTopology, RemoveUnreferenced-
// Verts, CalculateBBox, SortGeometry, IncrementMeshIDs).
func (nb *nativeBoolean3) Result(op OpType) *Manifold {
	// Empty-input early exits (boolean_result.cpp:743-753).
	pEmpty := nb.inP.NumTri() == 0
	qEmpty := nb.inQ.NumTri() == 0
	if pEmpty {
		if !qEmpty && op == OpAdd {
			return nb.inQ.Copy().ToManifold()
		}
		return newImpl().ToManifold()
	} else if qEmpty {
		if op == OpIntersect {
			return newImpl().ToManifold()
		}
		return nb.inP.Copy().ToManifold()
	}
	if !nb.b3.Valid() {
		// boolean_result.cpp:770 ResultTooLarge. Status propagation lands with
		// the production Manifold.Boolean rewire; until then surface it loudly.
		panic("nativeBoolean3.Result: ResultTooLarge (INT_MAX crossing overflow)")
	}

	rm, ok := nb.b3.Result(int(op))
	if !ok {
		return newImpl().ToManifold() // numVertR == 0
	}
	invertQ := op == OpSubtract

	// Build the output Impl from the triangulated ResultMesh.
	mi := newImpl()
	mi.h.ResizeVerts(len(rm.VertPos))
	copy(mi.Verts(), rm.VertPos)
	mi.h.SetHalfedgesRaw(rm.Starts, rm.PropVert, rm.Pairs)
	mi.h.ResizeFaceNormals(len(rm.TriNormal))
	copy(mi.h.FaceNormalsMut(), rm.TriNormal)
	setBoolTriRefs(mi, rm.TriRef)
	mi.h.SetEpsilonValue(rm.Epsilon)
	mi.h.SetToleranceValue(rm.Tolerance)

	// Finalize tail (boolean_result.cpp:951-972).
	createProperties(mi, nb.inP, nb.inQ, invertQ)
	updateReference(mi, nb.inP, nb.inQ, invertQ)
	mi.SimplifyTopology(rm.NPvNQv)
	mi.RemoveUnreferencedVerts()
	mi.CalculateBBox()
	mi.SortGeometry()
	mi.IncrementMeshIDs()
	return mi.ToManifold()
}

// setBoolTriRefs writes the assembled (boolean.TriRef) refs into outR's
// meshRelation_.triRef — the {meshID 0/1, faceID = original P/Q face} form
// Face2Tri produced; UpdateReference remaps it to the real input refs.
func setBoolTriRefs(outR *MutableImpl, refs []boolean.TriRef) {
	meshIDs := make([]int32, len(refs))
	originalIDs := make([]int32, len(refs))
	faceIDs := make([]int32, len(refs))
	coplanarIDs := make([]int32, len(refs))
	for i, r := range refs {
		meshIDs[i] = int32(r.MeshID)
		originalIDs[i] = int32(r.OriginalID)
		faceIDs[i] = int32(r.FaceID)
		coplanarIDs[i] = int32(r.CoplanarID)
	}
	outR.h.SetTriRefs(meshIDs, originalIDs, faceIDs, coplanarIDs)
}

// createProperties is the Go port of CreateProperties (boolean_result.cpp:571).
// Only the numProp==0 fast path is ported (set numProp_, return); the numProp>0
// barycentric interpolation is not yet ported and panics rather than silently
// emitting wrong properties.
func createProperties(outR *MutableImpl, inP, inQ *Impl, invertQ bool) {
	numProp := inP.Scalars().NumProp
	if q := inQ.Scalars().NumProp; q > numProp {
		numProp = q
	}
	outR.h.SetNumProp(numProp)
	if numProp == 0 {
		return
	}
	panic("createProperties: numProp>0 barycentric interpolation not yet ported")
}

// updateReference is the Go port of UpdateReference (boolean_result.cpp:518) and
// its MapTriRef functor (:505): remap each output tri's triRef from the
// {meshID 0/1, faceID = original P/Q face} form to the actual input triRef,
// offsetting Q's mesh IDs by the current meshIDCounter_; then copy both inputs'
// meshIDtransform entries into the output (Q offset, backSide XOR invertQ).
func updateReference(outR *MutableImpl, inP, inQ *Impl, invertQ bool) {
	offsetQ := int32(bridge.ImplReserveIDs(0)) // meshIDCounter_ (fetch_add 0)
	triRefP := inP.TriRefs()
	triRefQ := inQ.TriRefs()

	refs := outR.TriRefs()
	meshIDs := make([]int32, len(refs))
	originalIDs := make([]int32, len(refs))
	faceIDs := make([]int32, len(refs))
	coplanarIDs := make([]int32, len(refs))
	for i, ref := range refs {
		tri := int(ref.FaceID)
		pq := ref.MeshID == 0
		src := triRefP[tri]
		if !pq {
			src = triRefQ[tri]
			src.MeshID += offsetQ
		}
		meshIDs[i] = src.MeshID
		originalIDs[i] = src.OriginalID
		faceIDs[i] = src.FaceID
		coplanarIDs[i] = src.CoplanarID
	}
	outR.h.SetTriRefs(meshIDs, originalIDs, faceIDs, coplanarIDs)

	for _, r := range inP.MeshIDTransforms() {
		outR.h.AddMeshIDTransform(int(r.MeshID), int(r.OriginalID), r.Transform, r.BackSide, r.HasNormals)
	}
	for _, r := range inQ.MeshIDTransforms() {
		// meshIDtransform[pair.first + offsetQ] = pair.second; backSide ^= invertQ.
		outR.h.AddMeshIDTransform(int(r.MeshID)+int(offsetQ), int(r.OriginalID), r.Transform, r.BackSide != invertQ, r.HasNormals)
	}
}
