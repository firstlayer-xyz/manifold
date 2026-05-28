// Bridge shims exposing private C++ internals of libmanifold to the Go
// port. Each function here gives Go-side ports access to a layer that
// manifoldc's public C API hides. As inner layers move to Go, the shim
// surface shrinks accordingly.

#include "bridge.h"

#include <memory>

#include "manifold/manifold.h"
#include "manifold/polygon.h"

// Private headers from src/. Reachable via -I${SRCDIR}/../../src in the cgo
// CXXFLAGS. These pull in TBB (when MANIFOLD_PAR=1) via parallel.h.
#include "impl.h"
#include "csg_tree.h"

// from_c lives in bindings/c/conv.h; reachable via -I${SRCDIR}/../../bindings/c.
#include "conv.h"

// ManifoldBridge is friended by Manifold (see manifold.h). It is the
// designated way for Go-port cgo shims to reach internal C++ accessors
// without needing them to be public on the canonical class.
namespace manifold {
struct ManifoldBridge {
  static std::shared_ptr<const Manifold::Impl> GetImpl(const Manifold& m) {
    return m.GetCsgLeafNode().GetImpl();
  }
  static const int* HalfedgeStarts(const Halfedges& he) { return he.start_.data(); }
  static const int* HalfedgePairs(const Halfedges& he) { return he.paired_.data(); }
  static const int* HalfedgeProps(const Halfedges& he) { return he.propVert_.data(); }
  static void SetHalfedges(Halfedges& he, const int* starts, const int* props,
                           const int* paireds, size_t n) {
    he.clear(true);
    he.resize_nofill(n);
    for (size_t i = 0; i < n; ++i) {
      he.SetStart(static_cast<int>(i), starts[i]);
      he.SetProp(static_cast<int>(i), props[i]);
      he.SetPair(static_cast<int>(i), paireds[i]);
    }
  }
  static std::shared_ptr<CsgNode> LoadPNode(const Manifold& m) {
    return m.LoadPNode();
  }
  static Manifold FromCsgNode(std::shared_ptr<CsgNode> node) {
    return Manifold(std::move(node));
  }
};
}  // namespace manifold

using manifold::Manifold;

// Compile-time layout assertions for buffers exposed to Go via unsafe.Slice.
static_assert(sizeof(manifold::vec3) == 3 * sizeof(double),
              "vec3 must be 3 packed doubles for Go alias to be safe");
static_assert(alignof(manifold::vec3) == alignof(double),
              "vec3 alignment must match double for Go alias to be safe");
static_assert(sizeof(int) == 4, "halfedge start_ assumes 32-bit int");
using manifold::ManifoldBridge;

struct mb_impl_handle {
  std::shared_ptr<const Manifold::Impl> impl;
};

extern "C" {

mb_impl_handle* mb_get_impl(ManifoldManifold* m) {
  auto* mfld = from_c(m);
  return new mb_impl_handle{ManifoldBridge::GetImpl(*mfld)};
}

void mb_delete_impl(mb_impl_handle* h) { delete h; }

const void* mb_impl_vert_data(const mb_impl_handle* h, size_t* out_count) {
  const auto& vp = h->impl->vertPos_;
  *out_count = vp.size();
  return static_cast<const void*>(vp.data());
}

size_t mb_impl_halfedge_size(const mb_impl_handle* h) {
  return h->impl->halfedge_.size();
}

const void* mb_impl_halfedge_starts(const mb_impl_handle* h, size_t* out_count) {
  const auto& he = h->impl->halfedge_;
  *out_count = he.size();
  return static_cast<const void*>(ManifoldBridge::HalfedgeStarts(he));
}

const void* mb_impl_halfedge_props(const mb_impl_handle* h, size_t* out_count) {
  const auto& he = h->impl->halfedge_;
  *out_count = he.size();
  return static_cast<const void*>(ManifoldBridge::HalfedgeProps(he));
}

const double* mb_impl_halfedge_tangents(const mb_impl_handle* h,
                                         size_t* out_count) {
  const auto& ht = h->impl->halfedgeTangent_;
  *out_count = ht.size();
  return reinterpret_cast<const double*>(ht.data());
}

const double* mb_impl_properties(const mb_impl_handle* h, size_t* out_count) {
  const auto& p = h->impl->properties_;
  *out_count = p.size();
  return p.data();
}

void mb_impl_tri_refs(const mb_impl_handle* h, size_t* out_count,
                      const int** mesh_ids, const int** original_ids,
                      const int** face_ids, const int** coplanar_ids) {
  const auto& tr = h->impl->meshRelation_.triRef;
  *out_count = tr.size();
  if (tr.empty()) {
    *mesh_ids = nullptr;
    *original_ids = nullptr;
    *face_ids = nullptr;
    *coplanar_ids = nullptr;
    return;
  }
  *mesh_ids = &tr[0].meshID;
  *original_ids = &tr[0].originalID;
  *face_ids = &tr[0].faceID;
  *coplanar_ids = &tr[0].coplanarID;
}

size_t mb_impl_meshid_transform_count(const mb_impl_handle* h) {
  return h->impl->meshRelation_.meshIDtransform.size();
}

void mb_impl_meshid_transforms(const mb_impl_handle* h,
                                int* mesh_ids, int* original_ids,
                                double* transforms,
                                unsigned char* back_sides,
                                unsigned char* has_normals) {
  size_t i = 0;
  for (const auto& kv : h->impl->meshRelation_.meshIDtransform) {
    mesh_ids[i] = kv.first;
    original_ids[i] = kv.second.originalID;
    const auto& t = kv.second.transform;
    for (int col = 0; col < 4; ++col) {
      for (int row = 0; row < 3; ++row) {
        transforms[i * 12 + col * 3 + row] = t[col][row];
      }
    }
    back_sides[i] = kv.second.backSide ? 1 : 0;
    has_normals[i] = kv.second.hasNormals ? 1 : 0;
    ++i;
  }
}

void mb_impl_bbox(const mb_impl_handle* h,
                  double* min_x, double* min_y, double* min_z,
                  double* max_x, double* max_y, double* max_z) {
  const auto& b = h->impl->bBox_;
  *min_x = b.min.x; *min_y = b.min.y; *min_z = b.min.z;
  *max_x = b.max.x; *max_y = b.max.y; *max_z = b.max.z;
}

const void* mb_impl_halfedge_pairs(const mb_impl_handle* h, size_t* out_count) {
  const auto& he = h->impl->halfedge_;
  *out_count = he.size();
  return static_cast<const void*>(ManifoldBridge::HalfedgePairs(he));
}

const void* mb_impl_face_normals(const mb_impl_handle* h, size_t* out_count) {
  const auto& fn = h->impl->faceNormal_;
  *out_count = fn.size();
  return static_cast<const void*>(fn.data());
}

void mb_impl_get_scalars(const mb_impl_handle* h, mb_impl_scalars* out) {
  const auto& im = *h->impl;
  out->num_prop = static_cast<size_t>(im.numProp_);
  out->properties_size = im.properties_.size();
  out->halfedge_tangent_size = im.halfedgeTangent_.size();
  out->tolerance = im.tolerance_;
  out->epsilon = im.epsilon_;
  out->original_id = im.meshRelation_.originalID;
  out->status = static_cast<int>(im.status_);
}

struct mb_mutable_impl_handle {
  std::shared_ptr<Manifold::Impl> impl;
};

mb_mutable_impl_handle* mb_impl_copy(const mb_impl_handle* src) {
  return new mb_mutable_impl_handle{
      std::make_shared<Manifold::Impl>(*src->impl)};
}

mb_mutable_impl_handle* mb_new_mutable_impl(void) {
  return new mb_mutable_impl_handle{std::make_shared<Manifold::Impl>()};
}


double* mb_mutable_impl_verts_data(mb_mutable_impl_handle* h,
                                   size_t* out_count) {
  auto& vp = h->impl->vertPos_;
  *out_count = vp.size();
  return reinterpret_cast<double*>(vp.data());
}

void mb_mutable_impl_subdivide_n(mb_mutable_impl_handle* h, int n) {
  h->impl->Subdivide(
      [n](manifold::vec3, manifold::vec4, manifold::vec4) { return n - 1; });
}

void mb_mutable_impl_set_epsilon_min(mb_mutable_impl_handle* h,
                                     double minEpsilon) {
  h->impl->SetEpsilon(minEpsilon);
}

unsigned int mb_impl_reserve_ids(unsigned int n) {
  return Manifold::Impl::ReserveIDs(n);
}

void mb_mutable_impl_set_halfedges_raw(mb_mutable_impl_handle* h,
                                       const int* starts, const int* props,
                                       const int* paireds, size_t n) {
  ManifoldBridge::SetHalfedges(h->impl->halfedge_, starts, props, paireds, n);
}

void mb_mutable_impl_set_bbox(mb_mutable_impl_handle* h,
                              double min_x, double min_y, double min_z,
                              double max_x, double max_y, double max_z) {
  h->impl->bBox_.min = manifold::vec3(min_x, min_y, min_z);
  h->impl->bBox_.max = manifold::vec3(max_x, max_y, max_z);
}

void mb_mutable_impl_get_bbox(const mb_mutable_impl_handle* h,
                              double* min_x, double* min_y, double* min_z,
                              double* max_x, double* max_y, double* max_z) {
  *min_x = h->impl->bBox_.min.x;
  *min_y = h->impl->bBox_.min.y;
  *min_z = h->impl->bBox_.min.z;
  *max_x = h->impl->bBox_.max.x;
  *max_y = h->impl->bBox_.max.y;
  *max_z = h->impl->bBox_.max.z;
}

double mb_mutable_impl_get_tolerance(const mb_mutable_impl_handle* h) {
  return h->impl->tolerance_;
}

double mb_mutable_impl_get_epsilon(const mb_mutable_impl_handle* h) {
  return h->impl->epsilon_;
}

void mb_mutable_impl_set_meshrelation_original_id(mb_mutable_impl_handle* h,
                                                  int original_id) {
  h->impl->meshRelation_.originalID = original_id;
}

void mb_mutable_impl_set_tri_refs(mb_mutable_impl_handle* h,
                                  const int* meshIDs, const int* originalIDs,
                                  const int* faceIDs, const int* coplanarIDs,
                                  size_t num_tri) {
  auto& triRef = h->impl->meshRelation_.triRef;
  triRef.resize_nofill(num_tri);
  for (size_t i = 0; i < num_tri; ++i) {
    triRef[i] = manifold::TriRef{meshIDs[i], originalIDs[i], faceIDs[i],
                                  coplanarIDs[i]};
  }
}

void mb_mutable_impl_clear_meshid_transforms(mb_mutable_impl_handle* h) {
  h->impl->meshRelation_.meshIDtransform.clear();
}

void mb_mutable_impl_resize_face_normals(mb_mutable_impl_handle* h, size_t n) {
  h->impl->faceNormal_.resize(n);
}

double* mb_mutable_impl_face_normals_data(mb_mutable_impl_handle* h,
                                          size_t* out_count) {
  auto& fn = h->impl->faceNormal_;
  *out_count = fn.size();
  return reinterpret_cast<double*>(fn.data());
}

void mb_mutable_impl_set_coplanar_ids(mb_mutable_impl_handle* h,
                                      const int* coplanar_ids, size_t num_tri) {
  auto& triRef = h->impl->meshRelation_.triRef;
  for (size_t i = 0; i < num_tri; ++i) {
    triRef[i].coplanarID = coplanar_ids[i];
  }
}

const void* mb_mutable_impl_halfedge_starts(const mb_mutable_impl_handle* h,
                                            size_t* out_count) {
  const auto& he = h->impl->halfedge_;
  *out_count = he.size();
  return static_cast<const void*>(ManifoldBridge::HalfedgeStarts(he));
}

const void* mb_mutable_impl_halfedge_pairs(const mb_mutable_impl_handle* h,
                                           size_t* out_count) {
  const auto& he = h->impl->halfedge_;
  *out_count = he.size();
  return static_cast<const void*>(ManifoldBridge::HalfedgePairs(he));
}

const void* mb_mutable_impl_halfedge_props(const mb_mutable_impl_handle* h,
                                           size_t* out_count) {
  const auto& he = h->impl->halfedge_;
  *out_count = he.size();
  return static_cast<const void*>(ManifoldBridge::HalfedgeProps(he));
}

int mb_mutable_impl_num_prop(const mb_mutable_impl_handle* h) {
  return h->impl->numProp_;
}

const double* mb_mutable_impl_properties(const mb_mutable_impl_handle* h,
                                         size_t* out_count) {
  const auto& p = h->impl->properties_;
  *out_count = p.size();
  return p.data();
}

void mb_mutable_impl_tri_refs(const mb_mutable_impl_handle* h, size_t* out_count,
                              const int** mesh_ids, const int** original_ids,
                              const int** face_ids, const int** coplanar_ids) {
  const auto& tr = h->impl->meshRelation_.triRef;
  *out_count = tr.size();
  if (tr.empty()) {
    *mesh_ids = nullptr;
    *original_ids = nullptr;
    *face_ids = nullptr;
    *coplanar_ids = nullptr;
    return;
  }
  *mesh_ids = &tr[0].meshID;
  *original_ids = &tr[0].originalID;
  *face_ids = &tr[0].faceID;
  *coplanar_ids = &tr[0].coplanarID;
}

const double* mb_mutable_impl_halfedge_tangents(const mb_mutable_impl_handle* h,
                                                 size_t* out_count) {
  const auto& ht = h->impl->halfedgeTangent_;
  *out_count = ht.size();
  return reinterpret_cast<const double*>(ht.data());
}

size_t mb_mutable_impl_meshid_transform_count(const mb_mutable_impl_handle* h) {
  return h->impl->meshRelation_.meshIDtransform.size();
}

void mb_mutable_impl_meshid_transforms(const mb_mutable_impl_handle* h,
                                       int* mesh_ids, int* original_ids,
                                       double* transforms,
                                       unsigned char* back_sides,
                                       unsigned char* has_normals) {
  size_t i = 0;
  for (const auto& kv : h->impl->meshRelation_.meshIDtransform) {
    mesh_ids[i] = kv.first;
    original_ids[i] = kv.second.originalID;
    const auto& t = kv.second.transform;
    for (int col = 0; col < 4; ++col) {
      for (int row = 0; row < 3; ++row) {
        transforms[i * 12 + col * 3 + row] = t[col][row];
      }
    }
    back_sides[i] = kv.second.backSide ? 1 : 0;
    has_normals[i] = kv.second.hasNormals ? 1 : 0;
    ++i;
  }
}

void mb_mutable_impl_set_num_prop(mb_mutable_impl_handle* h, int num_prop) {
  h->impl->numProp_ = num_prop;
}

void mb_mutable_impl_set_properties_raw(mb_mutable_impl_handle* h,
                                        const double* data, size_t n) {
  auto& p = h->impl->properties_;
  p.resize_nofill(n);
  if (n > 0) std::copy(data, data + n, p.begin());
}

void mb_mutable_impl_set_halfedge_tangents_raw(mb_mutable_impl_handle* h,
                                                const double* data, size_t n) {
  auto& ht = h->impl->halfedgeTangent_;
  ht.resize_nofill(n);
  if (n > 0) {
    std::copy(reinterpret_cast<const manifold::vec4*>(data),
              reinterpret_cast<const manifold::vec4*>(data) + n, ht.begin());
  }
}

void mb_mutable_impl_add_meshid_transform(
    mb_mutable_impl_handle* h, int mesh_id, int original_id,
    double t00, double t01, double t02, double t10, double t11, double t12,
    double t20, double t21, double t22, double t30, double t31, double t32,
    int back_side, int has_normals) {
  manifold::mat3x4 t{
      manifold::vec3{t00, t01, t02},
      manifold::vec3{t10, t11, t12},
      manifold::vec3{t20, t21, t22},
      manifold::vec3{t30, t31, t32},
  };
  h->impl->meshRelation_.meshIDtransform[mesh_id] = {
      original_id, t, back_side != 0, has_normals != 0};
}

void mb_mutable_impl_make_empty(mb_mutable_impl_handle* h, int status) {
  h->impl->MakeEmpty(static_cast<manifold::Manifold::Error>(status));
}

void mb_mutable_impl_create_tangents_idx(mb_mutable_impl_handle* h,
                                         int normalIdx) {
  h->impl->CreateTangents(normalIdx);
}

void mb_mutable_impl_create_tangents_from_raw(mb_mutable_impl_handle* h,
                                              const size_t* halfedges,
                                              const double* smoothness,
                                              size_t n) {
  std::vector<manifold::Smoothness> sv;
  sv.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    sv.push_back({halfedges[i], smoothness[i]});
  }
  h->impl->CreateTangents(sv);
}

void mb_mutable_impl_set_normals(mb_mutable_impl_handle* h, int normalIdx,
                                 double minSharpAngle) {
  h->impl->SetNormals(normalIdx, minSharpAngle);
}

// (mb_polygons_handle + mb_impl_slice + mb_impl_project +
// mb_polygons_* accessors removed — Slice and Project are now
// native Go; see impl_slice.go and impl_project.go.)

// (mb_impl_get_meshgl / mb_impl_get_meshgl64 removed — GetMeshGL is
// implemented natively in Go via the impl read accessors above.)

struct mb_tri_verts_handle {
  manifold::Vec<manifold::ivec3> tris;
};

mb_tri_verts_handle* mb_triangulate(const double* cs_data,
                                    const size_t* cs_sizes,
                                    size_t num_polygons, double epsilon) {
  manifold::Polygons polys(num_polygons);
  size_t offset = 0;
  for (size_t i = 0; i < num_polygons; ++i) {
    size_t n = cs_sizes[i];
    polys[i].resize(n);
    const auto* p =
        reinterpret_cast<const manifold::vec2*>(cs_data + 2 * offset);
    std::copy(p, p + n, polys[i].begin());
    offset += n;
  }
  auto vec = manifold::Triangulate(polys, epsilon);
  auto* h = new mb_tri_verts_handle{};
  h->tris.resize(vec.size());
  std::copy(vec.begin(), vec.end(), h->tris.begin());
  return h;
}

size_t mb_tri_verts_count(const mb_tri_verts_handle* h) {
  return h->tris.size();
}

const int* mb_tri_verts_data(const mb_tri_verts_handle* h) {
  return reinterpret_cast<const int*>(h->tris.data());
}

void mb_delete_tri_verts(mb_tri_verts_handle* h) { delete h; }

}  // extern "C"

// _cgo_export.h is generated from the //export comments in bridge.go;
// it provides C declarations for the Go-callable trampolines.
#include "_cgo_export.h"

extern "C" {

void mb_mutable_impl_warp(mb_mutable_impl_handle* h, uintptr_t callback_id) {
  h->impl->Warp([callback_id](manifold::vec3& v) {
    double xyz[3] = {v.x, v.y, v.z};
    mbWarpTrampoline(static_cast<GoUintptr>(callback_id), xyz);
    v.x = xyz[0];
    v.y = xyz[1];
    v.z = xyz[2];
  });
}

void mb_mutable_impl_warp_batch(mb_mutable_impl_handle* h,
                                uintptr_t callback_id) {
  h->impl->WarpBatch([callback_id](manifold::VecView<manifold::vec3> arr) {
    mbWarpBatchTrampoline(static_cast<GoUintptr>(callback_id),
                          reinterpret_cast<double*>(arr.data()),
                          static_cast<size_t>(arr.size()));
  });
}

int mb_impl_matches_tri_normals(const mb_impl_handle* h) {
  return h->impl->MatchesTriNormals() ? 1 : 0;
}

int mb_impl_num_degenerate_tris(const mb_impl_handle* h) {
  return h->impl->NumDegenerateTris();
}

}  // extern "C"

#include "boolean3.h"

extern "C" {

struct mb_boolean3_handle {
  manifold::Boolean3 b3;
  mb_boolean3_handle(const manifold::Manifold::Impl& a,
                     const manifold::Manifold::Impl& b, manifold::OpType op)
      : b3(a, b, op) {}
};

mb_boolean3_handle* mb_boolean3_new(const mb_impl_handle* a,
                                    const mb_impl_handle* b, int op) {
  return new mb_boolean3_handle(*a->impl, *b->impl,
                                static_cast<manifold::OpType>(op));
}

ManifoldManifold* mb_boolean3_result_as_manifold(mb_boolean3_handle* h, int op) {
  auto resImpl = std::make_shared<manifold::Manifold::Impl>(
      h->b3.Result(static_cast<manifold::OpType>(op)));
  return to_c(new Manifold(Manifold::FromImpl(resImpl)));
}

void mb_delete_boolean3(mb_boolean3_handle* h) { delete h; }

ManifoldRayHitVec* mb_impl_ray_cast(const mb_impl_handle* h, double ox,
                                    double oy, double oz, double ex, double ey,
                                    double ez) {
  auto* hits = new RayHitVec(
      h->impl->RayCast(manifold::vec3(ox, oy, oz), manifold::vec3(ex, ey, ez)));
  return to_c(hits);
}

#define MB_ASSIGN_OPT(dst, src, n) \
  do {                            \
    if ((n) > 0 && (src) != nullptr) (dst).assign((src), (src) + (n)); \
  } while (0)

ManifoldManifold* mb_manifold_from_meshgl64(
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
    double tolerance) {
  manifold::MeshGL64 mesh;
  mesh.numProp = num_prop;
  MB_ASSIGN_OPT(mesh.vertProperties, vert_properties, vert_properties_len);
  MB_ASSIGN_OPT(mesh.triVerts, tri_verts, tri_verts_len);
  MB_ASSIGN_OPT(mesh.mergeFromVert, merge_from_vert, merge_from_vert_len);
  MB_ASSIGN_OPT(mesh.mergeToVert, merge_to_vert, merge_to_vert_len);
  MB_ASSIGN_OPT(mesh.runIndex, run_index, run_index_len);
  MB_ASSIGN_OPT(mesh.runOriginalID, run_original_id, run_original_id_len);
  MB_ASSIGN_OPT(mesh.runTransform, run_transform, run_transform_len);
  MB_ASSIGN_OPT(mesh.runFlags, run_flags, run_flags_len);
  MB_ASSIGN_OPT(mesh.faceID, face_id, face_id_len);
  MB_ASSIGN_OPT(mesh.halfedgeTangent, halfedge_tangent, halfedge_tangent_len);
  mesh.tolerance = tolerance;
  return to_c(new Manifold(mesh));
}

ManifoldManifold* mb_manifold_from_meshgl(
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
    float tolerance) {
  manifold::MeshGL mesh;
  mesh.numProp = num_prop;
  MB_ASSIGN_OPT(mesh.vertProperties, vert_properties, vert_properties_len);
  MB_ASSIGN_OPT(mesh.triVerts, tri_verts, tri_verts_len);
  MB_ASSIGN_OPT(mesh.mergeFromVert, merge_from_vert, merge_from_vert_len);
  MB_ASSIGN_OPT(mesh.mergeToVert, merge_to_vert, merge_to_vert_len);
  MB_ASSIGN_OPT(mesh.runIndex, run_index, run_index_len);
  MB_ASSIGN_OPT(mesh.runOriginalID, run_original_id, run_original_id_len);
  MB_ASSIGN_OPT(mesh.runTransform, run_transform, run_transform_len);
  MB_ASSIGN_OPT(mesh.runFlags, run_flags, run_flags_len);
  MB_ASSIGN_OPT(mesh.faceID, face_id, face_id_len);
  MB_ASSIGN_OPT(mesh.halfedgeTangent, halfedge_tangent, halfedge_tangent_len);
  mesh.tolerance = tolerance;
  return to_c(new Manifold(mesh));
}

ManifoldManifold* mb_manifold_smooth_meshgl64(
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
    size_t sharpened_len) {
  manifold::MeshGL64 mesh;
  mesh.numProp = num_prop;
  MB_ASSIGN_OPT(mesh.vertProperties, vert_properties, vert_properties_len);
  MB_ASSIGN_OPT(mesh.triVerts, tri_verts, tri_verts_len);
  MB_ASSIGN_OPT(mesh.mergeFromVert, merge_from_vert, merge_from_vert_len);
  MB_ASSIGN_OPT(mesh.mergeToVert, merge_to_vert, merge_to_vert_len);
  MB_ASSIGN_OPT(mesh.runIndex, run_index, run_index_len);
  MB_ASSIGN_OPT(mesh.runOriginalID, run_original_id, run_original_id_len);
  MB_ASSIGN_OPT(mesh.runTransform, run_transform, run_transform_len);
  MB_ASSIGN_OPT(mesh.runFlags, run_flags, run_flags_len);
  MB_ASSIGN_OPT(mesh.faceID, face_id, face_id_len);
  MB_ASSIGN_OPT(mesh.halfedgeTangent, halfedge_tangent, halfedge_tangent_len);
  mesh.tolerance = tolerance;

  std::vector<manifold::Smoothness> edges;
  edges.reserve(sharpened_len);
  for (size_t i = 0; i < sharpened_len; ++i) {
    edges.push_back({sharpened_halfedge[i], sharpened_smoothness[i]});
  }
  return to_c(new Manifold(Manifold::Smooth(mesh, edges)));
}

ManifoldManifold* mb_manifold_smooth_meshgl(
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
    size_t sharpened_len) {
  manifold::MeshGL mesh;
  mesh.numProp = num_prop;
  MB_ASSIGN_OPT(mesh.vertProperties, vert_properties, vert_properties_len);
  MB_ASSIGN_OPT(mesh.triVerts, tri_verts, tri_verts_len);
  MB_ASSIGN_OPT(mesh.mergeFromVert, merge_from_vert, merge_from_vert_len);
  MB_ASSIGN_OPT(mesh.mergeToVert, merge_to_vert, merge_to_vert_len);
  MB_ASSIGN_OPT(mesh.runIndex, run_index, run_index_len);
  MB_ASSIGN_OPT(mesh.runOriginalID, run_original_id, run_original_id_len);
  MB_ASSIGN_OPT(mesh.runTransform, run_transform, run_transform_len);
  MB_ASSIGN_OPT(mesh.runFlags, run_flags, run_flags_len);
  MB_ASSIGN_OPT(mesh.faceID, face_id, face_id_len);
  MB_ASSIGN_OPT(mesh.halfedgeTangent, halfedge_tangent, halfedge_tangent_len);
  mesh.tolerance = tolerance;

  std::vector<manifold::Smoothness> edges;
  edges.reserve(sharpened_len);
  for (size_t i = 0; i < sharpened_len; ++i) {
    edges.push_back({sharpened_halfedge[i], sharpened_smoothness[i]});
  }
  return to_c(new Manifold(Manifold::Smooth(mesh, edges)));
}

#undef MB_ASSIGN_OPT

void mb_mutable_impl_set_epsilon_value(mb_mutable_impl_handle* h,
                                       double epsilon) {
  h->impl->epsilon_ = epsilon;
}

void mb_mutable_impl_resize_verts(mb_mutable_impl_handle* h, size_t n) {
  h->impl->vertPos_.resize(n);
}

void mb_mutable_impl_resize_vert_normals(mb_mutable_impl_handle* h, size_t n) {
  h->impl->vertNormal_.resize(n);
}

double* mb_mutable_impl_vert_normals_data(mb_mutable_impl_handle* h,
                                          size_t* out_count) {
  auto& v = h->impl->vertNormal_;
  *out_count = v.size();
  return reinterpret_cast<double*>(v.data());
}

const double* mb_impl_vert_normals_data(const mb_impl_handle* h,
                                        size_t* out_count) {
  const auto& v = h->impl->vertNormal_;
  *out_count = v.size();
  return reinterpret_cast<const double*>(v.data());
}


void mb_mutable_impl_refine_n(mb_mutable_impl_handle* h, int n) {
  // Mirror of the lambda in C++ Manifold::Refine: constant n-1 splits.
  h->impl->Refine(
      [n](manifold::vec3, manifold::vec4, manifold::vec4) { return n - 1; },
      false);
}

void mb_mutable_impl_refine_to_length(mb_mutable_impl_handle* h, double length) {
  h->impl->Refine(
      [length](manifold::vec3 edge, manifold::vec4, manifold::vec4) {
        return static_cast<int>(la::length(edge) / length);
      },
      false);
}

ManifoldManifold* mb_impl_minkowski(const mb_impl_handle* a,
                                    const mb_impl_handle* b, int inset) {
  Manifold result = a->impl->Minkowski(*b->impl, inset != 0);
  return to_c(new Manifold(std::move(result)));
}

void mb_mutable_impl_refine_to_tolerance(mb_mutable_impl_handle* h,
                                         double tolerance) {
  h->impl->Refine(
      [tolerance](manifold::vec3 edge, manifold::vec4 tangentStart,
                  manifold::vec4 tangentEnd) {
        const manifold::vec3 edgeNorm = la::normalize(edge);
        const manifold::vec3 tStart = manifold::vec3(tangentStart);
        const manifold::vec3 tEnd = manifold::vec3(tangentEnd);
        const manifold::vec3 start =
            tStart - edgeNorm * la::dot(edgeNorm, tStart);
        const manifold::vec3 end = tEnd - edgeNorm * la::dot(edgeNorm, tEnd);
        const double d = 0.5 * (la::length(start) + la::length(end)) +
                         la::length(start - end);
        return static_cast<int>(std::sqrt(3 * d / (4 * tolerance)));
      },
      true);
}

void mb_mutable_impl_simplify_topology(mb_mutable_impl_handle* h) {
  h->impl->SimplifyTopology();
}

void mb_mutable_impl_build_collider(mb_mutable_impl_handle* h,
                                    const double* boxes, const uint32_t* morton,
                                    size_t n) {
  auto& impl = *h->impl;
  if (n == 0) {
    impl.collider_ = manifold::Collider{};
    return;
  }
  manifold::Vec<manifold::Box> faceBox(n);
  manifold::Vec<uint32_t> faceMorton(n);
  for (size_t i = 0; i < n; ++i) {
    faceBox[i].min = manifold::vec3(boxes[6*i+0], boxes[6*i+1], boxes[6*i+2]);
    faceBox[i].max = manifold::vec3(boxes[6*i+3], boxes[6*i+4], boxes[6*i+5]);
    faceMorton[i] = morton[i];
  }
  impl.collider_ = manifold::Collider(faceBox, faceMorton);
  impl.bBox_ = impl.collider_.GetBoundingBox();
  // CompactProps is now done Go-side; see impl_compact.go.
}

void mb_mutable_impl_collider_transform(
    mb_mutable_impl_handle* h,
    double t00, double t01, double t02, double t10, double t11, double t12,
    double t20, double t21, double t22, double t30, double t31, double t32) {
  manifold::mat3x4 t{
      manifold::vec3{t00, t01, t02},
      manifold::vec3{t10, t11, t12},
      manifold::vec3{t20, t21, t22},
      manifold::vec3{t30, t31, t32},
  };
  h->impl->collider_.Transform(t);
}

void mb_mutable_impl_collider_update_boxes(mb_mutable_impl_handle* h,
                                           const double* boxes, size_t n) {
  manifold::Vec<manifold::Box> faceBox(n);
  for (size_t i = 0; i < n; ++i) {
    faceBox[i].min = manifold::vec3(boxes[6 * i + 0], boxes[6 * i + 1], boxes[6 * i + 2]);
    faceBox[i].max = manifold::vec3(boxes[6 * i + 3], boxes[6 * i + 4], boxes[6 * i + 5]);
  }
  h->impl->collider_.UpdateBoxes(faceBox);
}

void mb_mutable_impl_set_tolerance_value(mb_mutable_impl_handle* h, double tol) {
  h->impl->tolerance_ = tol;
}

ManifoldManifold* mb_manifold_from_mutable_impl(mb_mutable_impl_handle* h) {
  return to_c(new Manifold(Manifold::FromImpl(h->impl)));
}

void mb_delete_mutable_impl(mb_mutable_impl_handle* h) { delete h; }

struct mb_csg_node_handle {
  std::shared_ptr<manifold::CsgNode> node;
};

mb_csg_node_handle* mb_manifold_load_pnode(ManifoldManifold* m) {
  auto* mfld = from_c(m);
  return new mb_csg_node_handle{ManifoldBridge::LoadPNode(*mfld)};
}

mb_csg_node_handle* mb_csg_node_transform(
    const mb_csg_node_handle* node,
    double x1, double y1, double z1,
    double x2, double y2, double z2,
    double x3, double y3, double z3,
    double x4, double y4, double z4) {
  manifold::mat3x4 m{
      manifold::vec3{x1, y1, z1},
      manifold::vec3{x2, y2, z2},
      manifold::vec3{x3, y3, z3},
      manifold::vec3{x4, y4, z4},
  };
  return new mb_csg_node_handle{node->node->Transform(m)};
}

mb_csg_node_handle* mb_csg_node_boolean(mb_csg_node_handle* a,
                                        const mb_csg_node_handle* b, int op) {
  return new mb_csg_node_handle{
      a->node->Boolean(b->node, static_cast<manifold::OpType>(op))};
}

mb_csg_node_handle* mb_csg_op_node(mb_csg_node_handle* const* nodes,
                                   size_t count, int op) {
  std::vector<std::shared_ptr<manifold::CsgNode>> children;
  children.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    children.push_back(nodes[i]->node);
  }
  return new mb_csg_node_handle{std::make_shared<manifold::CsgOpNode>(
      children, static_cast<manifold::OpType>(op))};
}

ManifoldManifold* mb_manifold_from_csg_node(mb_csg_node_handle* node) {
  return to_c(new Manifold(ManifoldBridge::FromCsgNode(node->node)));
}

void mb_delete_csg_node(mb_csg_node_handle* node) { delete node; }


}  // extern "C"
