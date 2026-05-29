package manifold

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
)

// validateMeshGL is the Go port of the validation cascade at the head of
// the C++ ingest constructor Manifold::Impl::Impl(const MeshGLP&)
// (src/impl.h:281-345). It returns the Error to report (via MakeEmpty) and
// proceed=false when the mesh is empty or malformed; proceed=true means the
// mesh passed every cascade check and geometry construction may continue.
//
// Faithfulness notes:
//   - The entry-time Cancelled gate (impl.h:281) is omitted: the Go port
//     models ctx as nil = never cancelled.
//   - numVert / numTri are computed exactly as C++ (integer division of the
//     raw flat buffers) before the numProp<3 guard. We additionally guard
//     numProp<=0 to avoid a divide-by-zero panic where C++ would be UB; for
//     realistic input (numProp>=3) the result is identical.
//   - isfinite rejects BOTH NaN and ±Inf, checked in the input's native
//     precision (float32 for MeshGL). It runs over the RAW flat buffers —
//     this is NOT the same as Impl.IsFinite() (which checks vertPos_ after
//     the copy and only the positions).
//   - MergeIndexOutOfBounds and VertexOutOfBounds are NOT checked here: C++
//     performs them later (prop2vert / triVert construction). Until those
//     steps are drilled the bridge body reports them.
func validateMeshGL[P float32 | float64, I uint32 | uint64](
	numProp int,
	vertProperties []P,
	triVerts []I,
	mergeFromVert, mergeToVert []I,
	runIndex []I,
	runOriginalID []uint32,
	runTransform []P,
	faceID []I,
	halfedgeTangent []P,
) (Error, bool) {
	numVert := 0
	if numProp > 0 {
		numVert = len(vertProperties) / numProp
	}
	numTri := len(triVerts) / 3

	if numVert == 0 && numTri == 0 {
		return NoError, false
	}
	if numVert < 4 || numTri < 4 {
		return NotManifold, false
	}
	if numProp < 3 {
		return MissingPositionProperties, false
	}
	if len(mergeFromVert) != len(mergeToVert) {
		return MergeVectorsDifferentLengths, false
	}
	if len(runTransform) > 0 && 12*len(runOriginalID) != len(runTransform) {
		return TransformWrongLength, false
	}
	if len(runOriginalID) > 0 && len(runIndex) > 0 &&
		len(runOriginalID)+1 != len(runIndex) && len(runOriginalID) != len(runIndex) {
		return RunIndexWrongLength, false
	}
	if len(faceID) > 0 && len(faceID) != numTri {
		return FaceIDWrongLength, false
	}

	finite := func(x P) bool { f := float64(x); return !math.IsNaN(f) && !math.IsInf(f, 0) }
	if !parallel.AllOf(parallel.AutoPolicy(len(vertProperties), 100000), len(vertProperties),
		func(i int) bool { return finite(vertProperties[i]) }) {
		return NonFiniteVertex, false
	}
	if !parallel.AllOf(parallel.AutoPolicy(len(runTransform), 100000), len(runTransform),
		func(i int) bool { return finite(runTransform[i]) }) {
		return InvalidConstruction, false
	}
	if !parallel.AllOf(parallel.AutoPolicy(len(halfedgeTangent), 100000), len(halfedgeTangent),
		func(i int) bool { return finite(halfedgeTangent[i]) }) {
		return InvalidConstruction, false
	}
	return NoError, true
}

// emptyManifold builds an empty Manifold carrying the given status —
// mirrors C++ MakeEmpty(status) followed by sealing into a Manifold.
func emptyManifold(status Error) *Manifold {
	mi := newImpl()
	defer mi.Delete()
	mi.h.MakeEmpty(int(status))
	return mi.ToManifold()
}
