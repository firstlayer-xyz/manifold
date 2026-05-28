// Custom C shims that expose private C++ internals of the manifold library
// to the Go port. Every function declared here exists because manifoldc
// does not expose the corresponding internal accessor.
//
// The Go port's outermost layer currently routes through these shims so
// that ports of public functions can call deeper into the library than
// manifoldc allows. As more layers move to Go, these shims grow (to expose
// more internals) and then shrink (as the things they expose become
// unnecessary).

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ManifoldManifold;

// Opaque handle to a manifold::Manifold::Impl. Holds a shared_ptr so the
// pointee outlives the original Manifold's lazy re-evaluations. Pair every
// mb_get_impl with mb_delete_impl.
typedef struct mb_impl_handle mb_impl_handle;

mb_impl_handle* mb_get_impl(struct ManifoldManifold* m);
void            mb_delete_impl(mb_impl_handle* h);

// mb_impl_vert_data returns a pointer to the first vec3 in the Impl's
// vertPos_ array and writes the element count through out_count. The
// underlying memory is owned by the Impl; the returned pointer is valid
// only while h is alive. Treat the array as read-only.
const void*     mb_impl_vert_data(const mb_impl_handle* h, size_t* out_count);

// mb_impl_halfedge_size returns the halfedge count, from which NumTri
// (size/3), NumEdge (size/2), and IsEmpty (size == 0) all derive.
size_t          mb_impl_halfedge_size(const mb_impl_handle* h);

// mb_impl_halfedge_starts returns a pointer to the start-vert array of
// the Impl's halfedges (the SoA start_ buffer of class Halfedges) and
// writes the element count via out_count. The array is read-only; valid
// only while h is alive. Triangle t has vertices at start[3t], start[3t+1],
// start[3t+2].
const void*     mb_impl_halfedge_starts(const mb_impl_handle* h, size_t* out_count);

// mb_impl_halfedge_props returns a read-only pointer to halfedge_.propVert_
// on a const Impl. Layout: int32_t[3*NumTri].
const void*     mb_impl_halfedge_props(const mb_impl_handle* h, size_t* out_count);

// mb_impl_halfedge_tangents returns a read-only pointer to
// halfedgeTangent_ on a const Impl. Layout: 4 packed doubles per
// element (manifold::vec4). out_count is the element count.
const double*   mb_impl_halfedge_tangents(const mb_impl_handle* h, size_t* out_count);

// mb_impl_properties returns a read-only pointer to properties_ on a
// const Impl. Layout: flat doubles, numProp_ entries per propVert.
// out_count is the total double count (= numProp_ * NumPropVert).
const double*   mb_impl_properties(const mb_impl_handle* h, size_t* out_count);

// mb_impl_tri_refs returns the four parallel arrays inside
// meshRelation_.triRef on a const Impl. All arrays have length NumTri.
// Each pointer points at the corresponding field of the TriRef struct
// at array index i (stride = sizeof(TriRef) = 4*sizeof(int)).
void            mb_impl_tri_refs(const mb_impl_handle* h, size_t* out_count,
    const int** mesh_ids, const int** original_ids,
    const int** face_ids, const int** coplanar_ids);

// mb_impl_meshid_transform_count returns the number of entries in
// meshRelation_.meshIDtransform.
size_t          mb_impl_meshid_transform_count(const mb_impl_handle* h);

// mb_impl_meshid_transforms iterates meshRelation_.meshIDtransform and
// writes the n entries into the caller's arrays. Order: increasing
// meshID (std::map iteration). All arrays must be at least count long
// (use mb_impl_meshid_transform_count first).
//
// transforms holds n * 12 doubles in column-major layout matching
// mb_mutable_impl_add_meshid_transform.
void            mb_impl_meshid_transforms(
    const mb_impl_handle* h,
    int* mesh_ids, int* original_ids,
    double* transforms,
    unsigned char* back_sides, unsigned char* has_normals);

// mb_impl_bbox reads the cached bBox_ (min/max corners) from a const Impl.
void            mb_impl_bbox(const mb_impl_handle* h,
    double* min_x, double* min_y, double* min_z,
    double* max_x, double* max_y, double* max_z);

// mb_impl_halfedge_pairs returns a pointer to the paired-edge array
// of the impl's Halfedges (halfedge_.paired_). Layout: int32_t[3*NumTri].
// Value -1 marks a boundary halfedge; otherwise the value is the index
// of the paired halfedge.
const void*     mb_impl_halfedge_pairs(const mb_impl_handle* h, size_t* out_count);

// mb_impl_face_normals returns a pointer to faceNormal_ (vec3 per
// triangle). Layout: 3 packed doubles per element. Empty when the impl
// has no cached face normals.
const void*     mb_impl_face_normals(const mb_impl_handle* h, size_t* out_count);

// mb_impl_scalars is a snapshot of small scalar fields on Manifold::Impl.
// Read together so the Go side can derive NumProp / NumPropVert /
// GetTolerance / OriginalID / Status without one cgo call per accessor.
typedef struct {
  size_t num_prop;
  size_t properties_size;
  size_t halfedge_tangent_size;
  double tolerance;
  double epsilon;
  int    original_id;
  int    status;
} mb_impl_scalars;

void mb_impl_get_scalars(const mb_impl_handle* h, mb_impl_scalars* out);

// Mutable Impl handle. mb_impl_copy returns a fresh shared_ptr<Impl>
// initialized from the source Impl (matching C++ make_shared<Impl>(*src)).
// Used by top-down ports that need to call non-const Impl methods. Pair
// every mb_impl_copy with mb_delete_mutable_impl.
typedef struct mb_mutable_impl_handle mb_mutable_impl_handle;

mb_mutable_impl_handle*  mb_impl_copy(const mb_impl_handle* src);
mb_mutable_impl_handle*  mb_new_mutable_impl(void);

// mb_mutable_impl_set_halfedges_raw writes the halfedge_ data wholesale
// from three parallel int32 arrays. Length n is the total halfedge
// count (== 3*NumTri); arrays must all be exactly n long. Replaces
// the current contents of halfedge_ (cleared first). Used by the
// Go port of CreateHalfedges.
void                     mb_mutable_impl_set_halfedges_raw(
    mb_mutable_impl_handle* h,
    const int* starts, const int* props, const int* paireds, size_t n);

// mb_mutable_impl_get_bbox reads bBox_.min and bBox_.max. Used by
// the Go port of SetEpsilon to compute MaxEpsilon(minEpsilon, bBox_).
void                     mb_mutable_impl_get_bbox(
    const mb_mutable_impl_handle* h,
    double* min_x, double* min_y, double* min_z,
    double* max_x, double* max_y, double* max_z);

// mb_mutable_impl_get_tolerance reads tolerance_, used by the Go
// port of SetEpsilon (which max-combines with the new minTol).
double                   mb_mutable_impl_get_tolerance(
    const mb_mutable_impl_handle* h);

// mb_mutable_impl_set_meshrelation_original_id assigns
// meshRelation_.originalID.
void                     mb_mutable_impl_set_meshrelation_original_id(
    mb_mutable_impl_handle* h, int original_id);

// mb_mutable_impl_set_tri_refs writes the meshRelation_.triRef array
// from four parallel int arrays (one entry per triangle). All four
// arrays must be exactly num_tri long.
void                     mb_mutable_impl_set_tri_refs(
    mb_mutable_impl_handle* h,
    const int* meshIDs, const int* originalIDs,
    const int* faceIDs, const int* coplanarIDs,
    size_t num_tri);

// mb_mutable_impl_clear_meshid_transforms removes all entries from
// meshRelation_.meshIDtransform.
void                     mb_mutable_impl_clear_meshid_transforms(
    mb_mutable_impl_handle* h);

// mb_mutable_impl_resize_face_normals resizes faceNormal_ to n
// elements. Used by the Go port of SetNormalsAndCoplanar before
// writing per-tri normals via the data accessor.
void                     mb_mutable_impl_resize_face_normals(
    mb_mutable_impl_handle* h, size_t n);

// mb_mutable_impl_face_normals_data returns a pointer to faceNormal_
// for direct Go-side write. Layout is 3 packed doubles per element
// (matching manifold::vec3). out_count gets the element count.
double*                  mb_mutable_impl_face_normals_data(
    mb_mutable_impl_handle* h, size_t* out_count);

// mb_mutable_impl_set_coplanar_ids writes only the coplanarID field of
// each TriRef in meshRelation_.triRef. The triRef array must already
// be the right size (== num_tri).
void                     mb_mutable_impl_set_coplanar_ids(
    mb_mutable_impl_handle* h, const int* coplanar_ids, size_t num_tri);

// mb_mutable_impl_halfedge_starts returns a read-only pointer to
// halfedge_.start_ on a mutable Impl (mirror of mb_impl_halfedge_starts
// for the const handle).
const void*              mb_mutable_impl_halfedge_starts(
    const mb_mutable_impl_handle* h, size_t* out_count);

// mb_mutable_impl_halfedge_pairs is the mutable-handle counterpart of
// mb_impl_halfedge_pairs.
const void*              mb_mutable_impl_halfedge_pairs(
    const mb_mutable_impl_handle* h, size_t* out_count);

// mb_mutable_impl_halfedge_props returns a read-only pointer to
// halfedge_.propVert_.
const void*              mb_mutable_impl_halfedge_props(
    const mb_mutable_impl_handle* h, size_t* out_count);

// mb_mutable_impl_num_prop reads numProp_ (per-vertex property count).
int                      mb_mutable_impl_num_prop(
    const mb_mutable_impl_handle* h);

// mb_mutable_impl_properties returns a read-only pointer to
// properties_ on a mutable Impl. Mirror of the const-Impl variant.
const double*            mb_mutable_impl_properties(
    const mb_mutable_impl_handle* h, size_t* out_count);

// mb_mutable_impl_tri_refs returns the four parallel arrays inside
// meshRelation_.triRef on a mutable Impl (mirror of the const-Impl
// variant).
void                     mb_mutable_impl_tri_refs(
    const mb_mutable_impl_handle* h, size_t* out_count,
    const int** mesh_ids, const int** original_ids,
    const int** face_ids, const int** coplanar_ids);

// mb_mutable_impl_halfedge_tangents returns a read-only pointer to
// halfedgeTangent_ on a mutable Impl. Layout: 4 packed doubles per
// element (vec4); out_count is the element count.
const double*            mb_mutable_impl_halfedge_tangents(
    const mb_mutable_impl_handle* h, size_t* out_count);

// mb_mutable_impl_meshid_transform_count returns the entry count of
// meshRelation_.meshIDtransform on a mutable Impl (mirror of the
// const-Impl variant).
size_t                   mb_mutable_impl_meshid_transform_count(
    const mb_mutable_impl_handle* h);

// mb_mutable_impl_meshid_transforms iterates the meshIDtransform map
// in std::map order (ascending meshID) and copies its entries into
// the caller's parallel arrays.
void                     mb_mutable_impl_meshid_transforms(
    const mb_mutable_impl_handle* h,
    int* mesh_ids, int* original_ids,
    double* transforms,
    unsigned char* back_sides, unsigned char* has_normals);

// mb_mutable_impl_set_num_prop writes numProp_. Pair with
// SetProperties below; the two must agree on count.
void                     mb_mutable_impl_set_num_prop(
    mb_mutable_impl_handle* h, int num_prop);

// mb_mutable_impl_set_properties_raw writes properties_ wholesale.
// data layout: n doubles total, interleaved numProp_ values per
// propVert (caller controls numProp_ via SetNumProp).
void                     mb_mutable_impl_set_properties_raw(
    mb_mutable_impl_handle* h, const double* data, size_t n);

// mb_mutable_impl_set_halfedge_tangents_raw writes halfedgeTangent_
// wholesale. data is 4*n doubles (one vec4 per halfedge); n=0 clears.
void                     mb_mutable_impl_set_halfedge_tangents_raw(
    mb_mutable_impl_handle* h, const double* data, size_t n);

// mb_mutable_impl_add_meshid_transform inserts one entry into
// meshRelation_.meshIDtransform. transform is a 12-double 3x4 affine
// matrix in column-major order (cols 0..2 linear, col 3 translation).
void                     mb_mutable_impl_add_meshid_transform(
    mb_mutable_impl_handle* h,
    int mesh_id, int original_id,
    double t00, double t01, double t02,
    double t10, double t11, double t12,
    double t20, double t21, double t22,
    double t30, double t31, double t32,
    int back_side, int has_normals);

// mb_mutable_impl_set_bbox writes bBox_.min and bBox_.max directly.
// Used by the Go port of Impl::CalculateBBox to install the
// Go-computed min/max corners.
void                     mb_mutable_impl_set_bbox(
    mb_mutable_impl_handle* h,
    double min_x, double min_y, double min_z,
    double max_x, double max_y, double max_z);

// mb_mutable_impl_make_empty wraps Impl::MakeEmpty(Error). status is
// cast to the C++ Error enum: 0=NoError, etc. (See Manifold::Error in
// manifold.h.)
void                     mb_mutable_impl_make_empty(
    mb_mutable_impl_handle* h, int status);

// (mb_invalid removed — Invalid is now native Go via MakeEmpty.)

// mb_mutable_impl_verts_data: mutable view onto the Impl's vertPos_
// array. Lets Go-side ports modify vertex positions in place. Same
// layout assumption as mb_impl_vert_data.
double*                  mb_mutable_impl_verts_data(
    mb_mutable_impl_handle* h, size_t* out_count);

// mb_mutable_impl_subdivide_n: Impl::Subdivide with the constant n-1
// splits-per-edge lambda (mirrors C++ Sphere's subdivision call).
void                     mb_mutable_impl_subdivide_n(
    mb_mutable_impl_handle* h, int n);

// mb_mutable_impl_set_epsilon_min: Impl::SetEpsilon(minEpsilon, false)
// — the one-arg overload used by ReadOBJ to restore the epsilon read
// from a `# epsilon = X` comment.
void                     mb_mutable_impl_set_epsilon_min(
    mb_mutable_impl_handle* h, double minEpsilon);

// (mb_quality_get_circular_segments, mb_mutable_impl_calculate_bbox,
// mb_mutable_impl_set_epsilon, mb_mutable_impl_initialize_original,
// and mb_mutable_impl_set_normals_and_coplanar were removed; these
// methods are now implemented natively in Go.)

// (mb_manifold_extrude removed — Manifold::Extrude is now native Go;
// see impl_extrude.go.)

// mb_impl_reserve_ids wraps Manifold::Impl::ReserveIDs(n) — reserves a
// block of n mesh IDs from the global counter.
unsigned int             mb_impl_reserve_ids(unsigned int n);

// mb_mutable_impl_create_tangents_idx: Impl::CreateTangents(normalIdx).
// Fills halfedgeTangent_ from per-vert normals at the given property slot.
void                     mb_mutable_impl_create_tangents_idx(
    mb_mutable_impl_handle* h, int normalIdx);

// Opaque handle around a std::vector<manifold::Smoothness>, returned by
// SharpenEdges and consumed by CreateTangents.
// mb_mutable_impl_create_tangents_from_raw takes parallel arrays
// halfedges[n] + smoothness[n] (matching manifold::Smoothness fields)
// and runs Impl::CreateTangents(vector<Smoothness>) with them. The
// caller owns the arrays.
void                      mb_mutable_impl_create_tangents_from_raw(
    mb_mutable_impl_handle* h, const size_t* halfedges,
    const double* smoothness, size_t n);

// (mb_mutable_impl_sharpen_edges, mb_smoothness_vec_handle, and
// mb_delete_smoothness_vec removed — SharpenEdges is now native Go;
// see impl_smoothing.go.)

// (mb_mutable_impl_calculate_curvature removed — CalculateCurvature
// is now native Go; see impl_curvature.go.)

// mb_mutable_impl_set_normals wraps Impl::SetNormals(normalIdx, minSharpAngle).
void                      mb_mutable_impl_set_normals(
    mb_mutable_impl_handle* h, int normalIdx, double minSharpAngle);

// (mb_mutable_impl_mark_all_meshid_has_normals removed — the
// iteration is now done in Go via the MeshIDTransforms accessor.)

// (mb_polygons_handle + mb_impl_slice + mb_impl_project removed —
// Slice and Project are now native Go.)

// (mb_impl_all_have_normals removed — now implemented in Go via the
// MeshIDTransforms accessor.)

// (mb_impl_get_meshgl64 / mb_impl_get_meshgl removed — see comment in
// bridge.cpp; GetMeshGL is implemented natively in Go.)

// mb_impl_ray_cast wraps Impl::RayCast(origin, endpoint), returning a
// ManifoldRayHitVec* compatible with the manifoldc accessor functions.
struct ManifoldRayHitVec* mb_impl_ray_cast(
    const mb_impl_handle* h,
    double ox, double oy, double oz,
    double ex, double ey, double ez);

// mb_manifold_from_meshgl64 wraps `Manifold(MeshGL64)`. Reassembles a
// C++ MeshGL64 from Go-owned arrays for every field, then constructs
// the Manifold. Optional arrays may be NULL with len == 0.
struct ManifoldManifold* mb_manifold_from_meshgl64(
    size_t num_prop,
    const double* vert_properties, size_t vert_properties_len,
    const uint64_t* tri_verts, size_t tri_verts_len,
    const uint64_t* merge_from_vert, size_t merge_from_vert_len,
    const uint64_t* merge_to_vert, size_t merge_to_vert_len,
    const uint64_t* run_index, size_t run_index_len,
    const uint32_t* run_original_id, size_t run_original_id_len,
    const double* run_transform, size_t run_transform_len,
    const uint8_t* run_flags, size_t run_flags_len,
    const uint64_t* face_id, size_t face_id_len,
    const double* halfedge_tangent, size_t halfedge_tangent_len,
    double tolerance);

// Triangulate wraps the public free function
// manifold::Triangulate(polygons, epsilon). Result is held in an opaque
// handle (vector<ivec3>); iterate via mb_tri_verts_count / _data.
typedef struct mb_tri_verts_handle mb_tri_verts_handle;

mb_tri_verts_handle* mb_triangulate(
    const double* cs_data, const size_t* cs_sizes, size_t num_polygons,
    double epsilon);
size_t              mb_tri_verts_count(const mb_tri_verts_handle* h);
const int*          mb_tri_verts_data(const mb_tri_verts_handle* h);
void                mb_delete_tri_verts(mb_tri_verts_handle* h);

// (mb_mutable_impl_create_halfedges removed — CreateHalfedges is now
// native Go; see impl_halfedges.go.)

// mb_mutable_impl_warp wraps Impl::Warp with a Go-side callback. The
// callback_id is opaque to C++; the lambda passes it back through the
// generated cgo trampoline (mbWarpTrampoline) along with the vec3 the
// Go callback should mutate in place.
void                mb_mutable_impl_warp(
    mb_mutable_impl_handle* h, uintptr_t callback_id);

// mb_mutable_impl_warp_batch wraps Impl::WarpBatch — the Go callback
// receives the entire vert buffer as a slice and may mutate in place.
void                mb_mutable_impl_warp_batch(
    mb_mutable_impl_handle* h, uintptr_t callback_id);

// mb_impl_matches_tri_normals wraps Impl::MatchesTriNormals().
int                 mb_impl_matches_tri_normals(const mb_impl_handle* h);

// mb_impl_num_degenerate_tris wraps Impl::NumDegenerateTris().
int                 mb_impl_num_degenerate_tris(const mb_impl_handle* h);

// Boolean3 wraps the private src/boolean3.h Boolean3 class. Used by
// Split to compute Intersect and Subtract results from a single
// underlying Boolean3 evaluation.
typedef struct mb_boolean3_handle mb_boolean3_handle;

mb_boolean3_handle* mb_boolean3_new(
    const mb_impl_handle* a, const mb_impl_handle* b, int op);
struct ManifoldManifold* mb_boolean3_result_as_manifold(
    mb_boolean3_handle* h, int op);
void                mb_delete_boolean3(mb_boolean3_handle* h);

// mb_manifold_smooth_meshgl64 wraps the static Manifold::Smooth(MeshGL64,
// sharpenedEdges). MeshGL64 is supplied as the same flat field layout
// as mb_manifold_from_meshgl64; sharpened edges are a parallel pair of
// arrays: halfedge_idx[i] and smoothness[i] form one Smoothness entry.
struct ManifoldManifold* mb_manifold_smooth_meshgl64(
    size_t num_prop,
    const double* vert_properties, size_t vert_properties_len,
    const uint64_t* tri_verts, size_t tri_verts_len,
    const uint64_t* merge_from_vert, size_t merge_from_vert_len,
    const uint64_t* merge_to_vert, size_t merge_to_vert_len,
    const uint64_t* run_index, size_t run_index_len,
    const uint32_t* run_original_id, size_t run_original_id_len,
    const double* run_transform, size_t run_transform_len,
    const uint8_t* run_flags, size_t run_flags_len,
    const uint64_t* face_id, size_t face_id_len,
    const double* halfedge_tangent, size_t halfedge_tangent_len,
    double tolerance,
    const size_t* sharpened_halfedge, const double* sharpened_smoothness,
    size_t sharpened_len);

// mb_manifold_smooth_meshgl: float/uint32 variant of Smooth.
struct ManifoldManifold* mb_manifold_smooth_meshgl(
    size_t num_prop,
    const float* vert_properties, size_t vert_properties_len,
    const uint32_t* tri_verts, size_t tri_verts_len,
    const uint32_t* merge_from_vert, size_t merge_from_vert_len,
    const uint32_t* merge_to_vert, size_t merge_to_vert_len,
    const uint32_t* run_index, size_t run_index_len,
    const uint32_t* run_original_id, size_t run_original_id_len,
    const float* run_transform, size_t run_transform_len,
    const uint8_t* run_flags, size_t run_flags_len,
    const uint32_t* face_id, size_t face_id_len,
    const float* halfedge_tangent, size_t halfedge_tangent_len,
    float tolerance,
    const size_t* sharpened_halfedge, const double* sharpened_smoothness,
    size_t sharpened_len);

// mb_manifold_from_meshgl: float/uint32 variant.
struct ManifoldManifold* mb_manifold_from_meshgl(
    size_t num_prop,
    const float* vert_properties, size_t vert_properties_len,
    const uint32_t* tri_verts, size_t tri_verts_len,
    const uint32_t* merge_from_vert, size_t merge_from_vert_len,
    const uint32_t* merge_to_vert, size_t merge_to_vert_len,
    const uint32_t* run_index, size_t run_index_len,
    const uint32_t* run_original_id, size_t run_original_id_len,
    const float* run_transform, size_t run_transform_len,
    const uint8_t* run_flags, size_t run_flags_len,
    const uint32_t* face_id, size_t face_id_len,
    const float* halfedge_tangent, size_t halfedge_tangent_len,
    float tolerance);

// (mb_disjoint_sets_* removed — DisjointSets is now implemented in
// pure Go; see go/disjoint_sets.go.)

// mb_mutable_impl_set_epsilon_value sets impl->epsilon_ directly (the
// epsilon-field assignment in Decompose's inherit-precision block).
void                      mb_mutable_impl_set_epsilon_value(
    mb_mutable_impl_handle* h, double epsilon);

// mb_mutable_impl_resize_verts / _vert_normals: resize the matching
// Vec<vec3> field. Used by Decompose before scatter-gather of the new
// component's vert data.
void                      mb_mutable_impl_resize_verts(
    mb_mutable_impl_handle* h, size_t n);
void                      mb_mutable_impl_resize_vert_normals(
    mb_mutable_impl_handle* h, size_t n);

// mb_mutable_impl_vert_normals_data: mutable view of vertNormal_.
double*                   mb_mutable_impl_vert_normals_data(
    mb_mutable_impl_handle* h, size_t* out_count);
// mb_impl_vert_normals_data: read-only view of vertNormal_.
const double*             mb_impl_vert_normals_data(
    const mb_impl_handle* h, size_t* out_count);

// (mb_mutable_impl_gather_faces removed — GatherFaces is now native
// Go; see impl_gather.go.)

void                     mb_mutable_impl_simplify_topology(mb_mutable_impl_handle* h);

// mb_mutable_impl_build_collider builds collider_ from the supplied
// faceBox + faceMorton arrays (already permuted into sorted order by
// the Go-side SortFaces), then refreshes bBox_ from the collider and
// runs CompactProps. boxes is laid out as n * 6 doubles (min.x, min.y,
// min.z, max.x, max.y, max.z per box).
void                     mb_mutable_impl_build_collider(
    mb_mutable_impl_handle* h,
    const double* boxes, const uint32_t* morton, size_t n);

// mb_mutable_impl_collider_transform applies Collider::Transform to the
// impl's existing collider_, reusing the radix tree (axis-aligned
// transforms only). Mirrors the axis-aligned branch of Impl::Transform's
// collider refresh. The 12 doubles are the mat3x4 in column-major order
// (4 columns of 3, same layout as mb_mutable_impl_add_meshid_transform).
void                     mb_mutable_impl_collider_transform(
    mb_mutable_impl_handle* h,
    double t00, double t01, double t02, double t10, double t11, double t12,
    double t20, double t21, double t22, double t30, double t31, double t32);

// mb_mutable_impl_collider_update_boxes recomputes the impl's collider_
// leaf boxes from the supplied per-face boxes, reusing the radix tree
// topology (Collider::UpdateBoxes). Mirrors the non-axis-aligned branch
// of Impl::Transform's collider refresh. boxes is n * 6 doubles
// (min.x, min.y, min.z, max.x, max.y, max.z per box).
void                     mb_mutable_impl_collider_update_boxes(
    mb_mutable_impl_handle* h, const double* boxes, size_t n);

void                     mb_mutable_impl_set_tolerance_value(mb_mutable_impl_handle* h, double tol);

// mb_mutable_impl_refine_n calls Impl::Refine with the C++ lambda that
// subdivides each edge into n pieces (constant n-1 splits per edge).
void                     mb_mutable_impl_refine_n(mb_mutable_impl_handle* h, int n);

// mb_mutable_impl_refine_to_length: Impl::Refine with the lambda that
// computes splits-per-edge as floor(|edge|/length).
void                     mb_mutable_impl_refine_to_length(
    mb_mutable_impl_handle* h, double length);

// mb_mutable_impl_refine_to_tolerance: Impl::Refine with the tangent-aware
// "fit to circular arc within tolerance" lambda.
void                     mb_mutable_impl_refine_to_tolerance(
    mb_mutable_impl_handle* h, double tolerance);

// mb_impl_minkowski calls Impl::Minkowski(other, inset), returning the
// resulting Manifold directly (Impl::Minkowski is the only Impl method
// that returns a Manifold; it allocates its own internals via Boolean).
struct ManifoldManifold* mb_impl_minkowski(
    const mb_impl_handle* a, const mb_impl_handle* b, int inset);

// Wraps the mutable Impl in a Manifold via Manifold::FromImpl. The Impl
// continues to be owned by both handles via shared_ptr; deleting the
// mutable handle does not affect the returned Manifold.
struct ManifoldManifold*  mb_manifold_from_mutable_impl(mb_mutable_impl_handle* h);
void                      mb_delete_mutable_impl(mb_mutable_impl_handle* h);

// (mb_propagate_status removed — PropagateStatus is now native Go via
// MakeEmpty.)

// Opaque handle around shared_ptr<CsgNode>. mb_manifold_load_pnode wraps
// the inner Manifold::LoadPNode (private; via ManifoldBridge).
// mb_csg_node_transform mirrors CsgNode::Transform(mat3x4) and returns a
// new CsgNode (typically a CsgOpNode). mb_manifold_from_csg_node wraps a
// CsgNode back into a Manifold via the private Manifold(shared_ptr<CsgNode>)
// ctor. Pair every Load/Transform with mb_delete_csg_node.
typedef struct mb_csg_node_handle mb_csg_node_handle;

mb_csg_node_handle* mb_manifold_load_pnode(struct ManifoldManifold* m);
mb_csg_node_handle* mb_csg_node_transform(
    const mb_csg_node_handle* node,
    double x1, double y1, double z1,
    double x2, double y2, double z2,
    double x3, double y3, double z3,
    double x4, double y4, double z4);

// mb_csg_node_boolean mirrors CsgNode::Boolean(second, op): builds a
// CsgOpNode combining two CsgNode trees under the given OpType
// (0=Add, 1=Subtract, 2=Intersect — matching the C++ enum class OpType).
mb_csg_node_handle* mb_csg_node_boolean(
    mb_csg_node_handle* a, const mb_csg_node_handle* b, int op);

// mb_csg_op_node wraps `make_shared<CsgOpNode>(children, op)` — combines
// many child CsgNode trees under a single op. nodes is a pointer to
// `count` handles. Used by BatchBoolean/Compose.
mb_csg_node_handle* mb_csg_op_node(
    mb_csg_node_handle* const* nodes, size_t count, int op);

struct ManifoldManifold* mb_manifold_from_csg_node(mb_csg_node_handle* node);
void                     mb_delete_csg_node(mb_csg_node_handle* node);

#ifdef __cplusplus
}
#endif
