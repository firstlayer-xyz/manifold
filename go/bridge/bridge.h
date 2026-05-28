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

// mb_new_impl_shape wraps the C++ Impl(Shape, mat3x4) constructor.
// shape: 0=Tetrahedron, 1=Cube, 2=Octahedron (mirroring enum class Shape).
// The matrix is the standard 3x4 affine transform (column-major).
mb_mutable_impl_handle*  mb_new_impl_shape(
    int shape,
    double x1, double y1, double z1,
    double x2, double y2, double z2,
    double x3, double y3, double z3,
    double x4, double y4, double z4);

// mb_invalid wraps Manifold::Invalid() (private static) — returns a
// Manifold with Error::InvalidConstruction status.
struct ManifoldManifold*  mb_invalid(void);

// mb_mutable_impl_verts_data: mutable view onto the Impl's vertPos_
// array. Lets Go-side ports modify vertex positions in place. Same
// layout assumption as mb_impl_vert_data.
double*                  mb_mutable_impl_verts_data(
    mb_mutable_impl_handle* h, size_t* out_count);

// mb_mutable_impl_subdivide_n: Impl::Subdivide with the constant n-1
// splits-per-edge lambda (mirrors C++ Sphere's subdivision call).
void                     mb_mutable_impl_subdivide_n(
    mb_mutable_impl_handle* h, int n);

// mb_mutable_impl_calculate_bbox: Impl::CalculateBBox — recomputes
// the Impl's cached bBox_ from vertPos_.
void                     mb_mutable_impl_calculate_bbox(
    mb_mutable_impl_handle* h);

// mb_mutable_impl_set_epsilon_min: Impl::SetEpsilon(minEpsilon, false)
// — the one-arg overload used by ReadOBJ to restore the epsilon read
// from a `# epsilon = X` comment.
void                     mb_mutable_impl_set_epsilon_min(
    mb_mutable_impl_handle* h, double minEpsilon);

// mb_mutable_impl_set_epsilon: Impl::SetEpsilon(-1, false) — default
// argument form used by Sphere/Cylinder/etc.
void                     mb_mutable_impl_set_epsilon(
    mb_mutable_impl_handle* h);

// mb_quality_get_circular_segments: Quality::GetCircularSegments(radius).
// Returns the default segment count for a given radius based on the
// global Quality params.
int                      mb_quality_get_circular_segments(double radius);

// mb_manifold_extrude wraps Manifold::Extrude(Polygons, height,
// nDivisions, twistDegrees, scaleTop). cs_data is a flat array of vec2
// points (sum(cs_sizes) total). cs_sizes[i] is the point count of
// polygon i.
struct ManifoldManifold*  mb_manifold_extrude(
    const double* cs_data, const size_t* cs_sizes, size_t num_polygons,
    double height, int n_divisions, double twist_degrees,
    double scale_x, double scale_y);

// mb_impl_reserve_ids wraps Manifold::Impl::ReserveIDs(n) — reserves a
// block of n mesh IDs from the global counter.
unsigned int             mb_impl_reserve_ids(unsigned int n);

// mb_mutable_impl_create_tangents_idx: Impl::CreateTangents(normalIdx).
// Fills halfedgeTangent_ from per-vert normals at the given property slot.
void                     mb_mutable_impl_create_tangents_idx(
    mb_mutable_impl_handle* h, int normalIdx);

// Opaque handle around a std::vector<manifold::Smoothness>, returned by
// SharpenEdges and consumed by CreateTangents.
typedef struct mb_smoothness_vec_handle mb_smoothness_vec_handle;

mb_smoothness_vec_handle* mb_mutable_impl_sharpen_edges(
    mb_mutable_impl_handle* h, double min_sharp_angle, double min_smoothness);
void                      mb_mutable_impl_create_tangents_from(
    mb_mutable_impl_handle* h, mb_smoothness_vec_handle* sv);
void                      mb_delete_smoothness_vec(mb_smoothness_vec_handle* sv);

// mb_mutable_impl_calculate_curvature wraps
// Impl::CalculateCurvature(gaussianIdx, meanIdx).
void                      mb_mutable_impl_calculate_curvature(
    mb_mutable_impl_handle* h, int gaussianIdx, int meanIdx);

// mb_mutable_impl_set_normals wraps Impl::SetNormals(normalIdx, minSharpAngle).
void                      mb_mutable_impl_set_normals(
    mb_mutable_impl_handle* h, int normalIdx, double minSharpAngle);

// mb_mutable_impl_mark_all_meshid_has_normals iterates
// meshRelation_.meshIDtransform and sets hasNormals = true on every
// entry. Encapsulates the C++ for-loop because the iteration uses a
// std::unordered_map and isn't a single statement we can cleanly
// translate without exposing the map.
void                      mb_mutable_impl_mark_all_meshid_has_normals(
    mb_mutable_impl_handle* h);

// Opaque handle around a manifold::Polygons (vector of SimplePolygon).
// Returned by Impl::Slice / Impl::Project, then iterated by Go to copy
// the data into Go-owned slices.
typedef struct mb_polygons_handle mb_polygons_handle;

mb_polygons_handle*       mb_impl_slice(const mb_impl_handle* h, double height);
mb_polygons_handle*       mb_impl_project(const mb_impl_handle* h);

// mb_impl_all_have_normals wraps Impl::AllHaveNormals().
int                       mb_impl_all_have_normals(const mb_impl_handle* h);

// mb_impl_get_meshgl64 wraps the templated free function
// GetMeshGLImpl<double, uint64_t>(impl, normalIdx). Returns a
// ManifoldMeshGL64* (compatible with all the manifoldc meshgl64 read
// accessors). Release with manifold_delete_meshgl64 via the matching Go
// wrapper.
struct ManifoldMeshGL64*  mb_impl_get_meshgl64(
    const mb_impl_handle* h, int normalIdx);

// mb_impl_get_meshgl wraps GetMeshGLImpl<float, uint32_t>(impl, normalIdx).
struct ManifoldMeshGL*    mb_impl_get_meshgl(
    const mb_impl_handle* h, int normalIdx);

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

// mb_mutable_impl_create_halfedges wraps Impl::CreateHalfedges with the
// supplied triangle indices (3 ints per triangle).
void                mb_mutable_impl_create_halfedges(
    mb_mutable_impl_handle* h, const int* tri_verts, size_t tri_count);

// mb_manifold_level_set wraps the static Manifold::LevelSet. The SDF
// is supplied as a Go-side callback referenced by id; mb_manifold_level_set
// builds a std::function<double(vec3)> that calls back into Go via
// mbLevelSetTrampoline. Pass can_parallel=0 unless the Go callback is
// proven safe to call from multiple TBB threads concurrently.
struct ManifoldManifold* mb_manifold_level_set(
    uintptr_t callback_id,
    double bbox_min_x, double bbox_min_y, double bbox_min_z,
    double bbox_max_x, double bbox_max_y, double bbox_max_z,
    double edge_length, double level, double tolerance,
    int can_parallel);

// mb_manifold_set_properties wraps Manifold::SetProperties. The Go
// callback is referenced by callback_id; pass 0 to invoke the C++
// nullptr-callback path (zero-fills new properties in parallel). The
// trampoline (mbSetPropertiesTrampoline) is called once per halfedge
// per triangle, receiving new/old property pointers + the vertex
// position.
struct ManifoldManifold* mb_manifold_set_properties(
    struct ManifoldManifold* m, int num_prop, uintptr_t callback_id);

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

// DisjointSets wraps the private src/disjoint_sets.h class — used by
// Decompose to union-find connected vert components.
typedef struct mb_disjoint_sets_handle mb_disjoint_sets_handle;

mb_disjoint_sets_handle*  mb_disjoint_sets_new(size_t size);
void                      mb_disjoint_sets_unite(
    mb_disjoint_sets_handle* h, size_t a, size_t b);
// connected_components fills out_components (caller buffer of size set by
// DisjointSets ctor) and returns the count.
int                       mb_disjoint_sets_connected_components(
    mb_disjoint_sets_handle* h, int* out_components);
void                      mb_delete_disjoint_sets(mb_disjoint_sets_handle* h);

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

// mb_mutable_impl_gather_faces: Impl::GatherFaces(old, faceNew2Old).
// Source is a const handle; dst is the mutable target.
void                      mb_mutable_impl_gather_faces(
    mb_mutable_impl_handle* dst, const mb_impl_handle* src,
    const int* faceNew2Old, size_t count);

// mb_mutable_impl_reindex_verts: Impl::ReindexVerts(vertNew2Old, numOldVert).
void                      mb_mutable_impl_reindex_verts(
    mb_mutable_impl_handle* h, const int* vertNew2Old, size_t count,
    size_t numOldVert);
size_t                    mb_polygons_num_polys(const mb_polygons_handle* p);
size_t                    mb_polygons_poly_size(const mb_polygons_handle* p,
                                                size_t idx);
const double*             mb_polygons_poly_data(const mb_polygons_handle* p,
                                                size_t idx);
void                      mb_delete_polygons(mb_polygons_handle* p);
void                     mb_mutable_impl_initialize_original(mb_mutable_impl_handle* h);
void                     mb_mutable_impl_set_normals_and_coplanar(mb_mutable_impl_handle* h);
void                     mb_mutable_impl_simplify_topology(mb_mutable_impl_handle* h);
void                     mb_mutable_impl_sort_geometry(mb_mutable_impl_handle* h);
void                     mb_mutable_impl_set_tolerance_value(mb_mutable_impl_handle* h, double tol);

// mb_mutable_impl_hull calls C++ Impl::Hull(vert_pos, ctx=nullptr) — fills
// the (empty) mutable Impl with the convex hull of the given vert array.
// vert_pos is count vec3s packed as count*3 doubles.
void                     mb_mutable_impl_hull(
    mb_mutable_impl_handle* h, const double* vert_pos, size_t count);

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

// mb_impl_min_gap calls Impl::MinGap(other, searchLength) — minimum
// surface distance, clamped to searchLength.
double mb_impl_min_gap(
    const mb_impl_handle* a, const mb_impl_handle* b, double searchLength);

// Wraps the mutable Impl in a Manifold via Manifold::FromImpl. The Impl
// continues to be owned by both handles via shared_ptr; deleting the
// mutable handle does not affect the returned Manifold.
struct ManifoldManifold*  mb_manifold_from_mutable_impl(mb_mutable_impl_handle* h);
void                      mb_delete_mutable_impl(mb_mutable_impl_handle* h);

// mb_propagate_status mirrors Manifold::PropagateStatus: builds an empty
// Impl with the given Error code and wraps it. Used by top-down ports
// that need to forward an error status from an input Manifold.
struct ManifoldManifold* mb_propagate_status(int status);

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
