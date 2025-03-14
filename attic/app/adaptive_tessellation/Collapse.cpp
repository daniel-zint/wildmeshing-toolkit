#include "Collapse.h"
#include "PairUtils.hpp"
using namespace wmtk;
using namespace adaptive_tessellation;

// every edge is collapsed, if it is shorter than 3/4 L

namespace {
constexpr static size_t dummy = std::numeric_limits<size_t>::max();
}

// sort by accuracy - order from smallest error to largest; (yunfan will add in barrier to collapse)
// perhaps try doing collapses that mitigate error the most
namespace {
auto renew = [](auto& m, auto op, auto& tris) {
    auto edges = m.new_edges_after(tris);
    auto optup = std::vector<std::pair<std::string, wmtk::TriMesh::Tuple>>();
    for (auto& e : edges) optup.emplace_back(op, e);
    return optup;
};

template <typename Executor>
void addPairedCustomOps(Executor& e)
{
    e.add_operation(std::make_shared<AdaptiveTessellationPairedEdgeCollapseOperation>());
}

template <typename Executor>
void addCustomOps(Executor& e)
{
    e.add_operation(std::make_shared<AdaptiveTessellationEdgeCollapseOperation>());
}

std::string_view constraint_type(
    AdaptiveTessellationEdgeCollapseOperation::ConstrainedBoundaryType cbt)
{
    using ConstrainedBoundaryType =
        AdaptiveTessellationEdgeCollapseOperation::ConstrainedBoundaryType;
    switch (cbt) {
    case ConstrainedBoundaryType::NoConstraints: return "Neither";
    case ConstrainedBoundaryType::TupleSideConstrained: return "TupleSide";
    case ConstrainedBoundaryType::OtherSideConstrained: return "OtherSide";
    case ConstrainedBoundaryType::BothConstrained: return "Both";
    }
    return "";
}
} // namespace

void AdaptiveTessellationEdgeCollapseOperation::store_merged_seam_data(
    const AdaptiveTessellation& m,
    const Tuple& edge_tuple)
{
    // edge_tuple is the edge that will be collapsed
    OpCache& op_cache = m_op_cache.local();
    auto& vertex_seam_data_map = op_cache.new_vertex_seam_data;
    auto& edge_seam_data_map = op_cache.opposing_edge_seam_data;

    const size_t v1 = op_cache.v1;
    const size_t v2 = op_cache.v2;
    assert(v1 != v2);

    std::optional<Tuple> collapse_mirror;
    if (m.is_seam_edge(edge_tuple)) {
        collapse_mirror = m.get_oriented_mirror_edge(edge_tuple);
    }
    // if exists, should be edge_tuple.vid == collapse_mirror_vertex.vid
    // edge_tuple.oriented_mirror points to the other vid so i have to switch
    std::optional<std::array<size_t, 2>> collapse_mirror_vids;
    if (collapse_mirror.has_value()) {
        const Tuple& mt = collapse_mirror.value();
        collapse_mirror_vids = std::array<size_t, 2>{{mt.vid(m), mt.switch_vertex(m).vid(m)}};
    }


    // helper to identify if two half-edges only differ by orientation
    auto is_same_face_edge = [&](const Tuple& a, const Tuple& b) -> bool {
        return a.fid(m) == b.fid(m) && a.local_eid(m) == b.local_eid(m);
    };

    auto edge_tuple_to_vids = [&](const Tuple& t) {
        std::array<size_t, 2> r{{t.vid(m), t.switch_vertex(m).vid(m)}};
        std::sort(r.begin(), r.end());
        return r;
    };

    const auto base_edge_vids = edge_tuple_to_vids(edge_tuple);

    // const skippable_edge = [&](const Tuple& e) {
    //     return base_edge_vids == edge_tuple_to_vids(e);

    //};
    // store edges that will be radial from the new vertex
    // assumes the vid is not one of the input edge vids
    // edge tuple pointing from the radial vertex to the center vertex
    auto store_new_vertex_edge = [&](const Tuple& radial_edge) {
        assert(radial_edge.vid(m) != v1 && radial_edge.vid(m) != v2);
        if (!m.is_boundary_edge(radial_edge)) {
            return;
        }
        std::optional<std::array<size_t, 2>> mirror_edge_vids_opt;
        if (m.is_seam_edge(radial_edge)) {
            const Tuple mt = m.get_oriented_mirror_edge(radial_edge);
            // vid that is not a mirror of the collapsed edge
            const size_t mirror_radial_vid = mt.switch_vertex(m).vid(m);
            if (collapse_mirror_vids.has_value()) {
                const size_t mirror_vid = mt.vid(m);
                const auto& [mv0, mv1] = collapse_mirror_vids.value();
                if (mirror_vid == mv0 || mirror_vid == mv1) {
                    mirror_edge_vids_opt =
                        std::array<size_t, 2>{{mirror_radial_vid, mirror_radial_vid}};
                }
            }
            if (!mirror_edge_vids_opt.has_value()) {
                mirror_edge_vids_opt = edge_tuple_to_vids(mt);
            }
        }

        const auto curveid_opt = m.get_edge_attrs(radial_edge).curve_id;
        assert(curveid_opt.has_value());

        vertex_seam_data_map[radial_edge.vid(m)] =
            SeamData{mirror_edge_vids_opt, static_cast<size_t>(curveid_opt.value())};
    };


    for (const Tuple& edge : m.get_one_ring_edges_for_vertex(edge_tuple)) {
        if (edge.vid(m) != v2) store_new_vertex_edge(edge);
    }

    for (const Tuple& edge : m.get_one_ring_edges_for_vertex(edge_tuple.switch_vertex(m))) {
        if (edge.vid(m) != v1) store_new_vertex_edge(edge);
    }


    // fids for triangles we dont want to use opposing edge with
    const std::vector<size_t> bad_opposing_fids = [&]() {
        const std::vector<Tuple> bad_opposing_tris = m.tris_bounded_by_edge(edge_tuple);
        std::vector<size_t> ret;
        std::transform(
            bad_opposing_tris.begin(),
            bad_opposing_tris.end(),
            std::back_inserter(ret),
            [&](const Tuple& t) -> size_t { return t.fid(m); });
        std::sort(ret.begin(), ret.end());
        return ret;
    }();

    const auto bad_face = [&](const Tuple& t) -> bool {
        size_t fid = t.fid(m);
        return std::binary_search(bad_opposing_fids.begin(), bad_opposing_fids.end(), fid);
    };

    // takes in a edge that does not includ either input edge vids
    auto store_opposing_edge = [&](const Tuple& edge_tuple) {
        if (!m.is_boundary_edge(edge_tuple)) {
            return;
        }
        if (bad_face(edge_tuple)) {
            return;
        }
        std::optional<std::array<size_t, 2>> mirror_edge_vids_opt;
        if (m.is_seam_edge(edge_tuple)) {
            const auto meopt = m.get_mirror_edge_opt(edge_tuple);
            assert(meopt.has_value());
            const Tuple& mt = meopt.value();
            const size_t mirror_vid0 = mt.vid(m);
            const size_t mirror_vid1 = mt.switch_vertex(m).vid(m);
            mirror_edge_vids_opt = std::array<size_t, 2>{{mirror_vid0, mirror_vid1}};
        }
        const auto [v0, v1] = edge_tuple_to_vids(edge_tuple);

        const auto curveid_opt = m.get_edge_attrs(edge_tuple).curve_id;
        assert(curveid_opt.has_value());
        edge_seam_data_map[std::array<size_t, 2>{{v0, v1}}] =
            SeamData{mirror_edge_vids_opt, static_cast<size_t>(curveid_opt.value())};
    };

    // store the 1 ring tris of a
    for (const Tuple& tri : m.get_one_ring_tris_for_vertex(edge_tuple)) {
        const Tuple e = tri.switch_vertex(m).switch_edge(m);
        size_t a = e.vid(m);
        size_t b = e.switch_vertex(m).vid(m);
        if (a != v2 && b != v2) store_opposing_edge(e);
    }
    for (const Tuple& tri : m.get_one_ring_tris_for_vertex(edge_tuple.switch_vertex(m))) {
        const Tuple e = tri.switch_vertex(m).switch_edge(m);
        size_t a = e.vid(m);
        size_t b = e.switch_vertex(m).vid(m);
        if (a != v1 && b != v1) store_opposing_edge(e);
    }


    // store the 1 ring tris of b

    /*
    // store A B according to previous diagram
    {
        const Tuple e1 = edge_tuple.switch_edge(m).switch_vertex(m);
        store_edge(e1);
        const Tuple e2 = e1.switch_edge(m);
        store_edge(e2);
    }
    {
        // Store C D according to previous
        if (auto face_opt = edge_tuple.switch_face(m); face_opt.has_value()) {
            const Tuple e1 = face_opt.value().switch_edge(m).switch_vertex(m);
            store_edge(e1);
            const Tuple e2 = e1.switch_edge(m);
            store_edge(e2);
        }
    }
    */
}
auto AdaptiveTessellationEdgeCollapseOperation::merge(
    ConstrainedBoundaryType a,
    ConstrainedBoundaryType b) -> ConstrainedBoundaryType
{
    // constraint is implemented as a bitmask so or hte bitmasks to get maximal constraintness
    return static_cast<ConstrainedBoundaryType>(static_cast<char>(a) | static_cast<char>(b));
}

auto AdaptiveTessellationEdgeCollapseOperation::get_constrained_boundary_type(
    const AdaptiveTessellation& m,
    const Tuple& t) const -> ConstrainedBoundaryType
{
    ConstrainedBoundaryType primary = get_constrained_boundary_type_per_face(m, t);
    if (const auto oface_opt = t.switch_face(m); oface_opt.has_value()) {
        ConstrainedBoundaryType secondary =
            get_constrained_boundary_type_per_face(m, oface_opt.value());

        auto m = merge(primary, secondary);
        return m;
    } else {
        return primary;
    }
}
auto AdaptiveTessellationEdgeCollapseOperation::get_constrained_boundary_type_per_face(
    const AdaptiveTessellation& m,
    const Tuple& t) const -> ConstrainedBoundaryType
{
    const Tuple other_tuple = t.switch_vertex(m);
    const auto& t_vattr = m.get_vertex_attrs(t);
    const auto& o_vattr = m.get_vertex_attrs(other_tuple);

    const auto& edge_curveid_opt = m.get_edge_attrs(t).curve_id;

    const bool edge_is_boundary = m.is_boundary_edge(t);
    // const bool vertex_same_vattrs = t_vattr.curve_id == o_vattr.curve_id;

    auto vertex_is_constrained = [&](const Tuple& v) -> bool {
        const auto& vattr = m.get_vertex_attrs(v);
        // fixed vertices can't move
        if (vattr.fixed) {
            return true;
        }

        // if a vertex is on boundary then the other one cannot be on a boundary unless it's on
        // an edge
        // TODO: make sure boundary vertices are properly handled
        if (edge_is_boundary) {
            return false;

        } else {
            if (m.is_boundary_vertex(v)) {
                return true;
            }
        }

        return false;
    };
    auto edge_is_constrained = [&](const Tuple& e) -> bool { return m.is_boundary_edge(e); };

    // t must be an edge tuple  pointing at the desired vertex on a particular face
    auto side_is_constrained = [&](const Tuple& edge) -> bool {
        const bool this_side_constrained =
            edge_is_constrained(edge.switch_edge(m)) || vertex_is_constrained(edge);

        auto edge2_opt = edge.switch_face(m);
        if (edge2_opt) {
            const Tuple& edge2 = edge2_opt.value();
            const bool other_edge_constrained = edge_is_constrained(edge2.switch_edge(m));
            return other_edge_constrained || this_side_constrained;
        } else {
            return this_side_constrained;
        }
    };


    const bool ts = side_is_constrained(t);


    const bool os = side_is_constrained(other_tuple);


    if (ts) {
        if (os) {
            return ConstrainedBoundaryType::BothConstrained;

        } else {
            return ConstrainedBoundaryType::TupleSideConstrained;
        }
    } else {
        if (os) {
            return ConstrainedBoundaryType::OtherSideConstrained;
        } else {
            return ConstrainedBoundaryType::NoConstraints;
        }
    }
    return ConstrainedBoundaryType::NoConstraints;
}

void AdaptiveTessellationEdgeCollapseOperation::fill_cache(
    const AdaptiveTessellation& m,
    const Tuple& t)
{
    OpCache& op_cache = m_op_cache.local();
    // check if the two vertices to be split is of the same curve_id
    const size_t my_vid = t.vid(m);
    const Tuple other_tuple = t.switch_vertex(m);
    const size_t other_vid = other_tuple.vid(m);
    auto& my_vattr = m.vertex_attrs[my_vid];
    auto& other_vattr = m.vertex_attrs[other_vid];


    // if (!m.mesh_parameters.m_ignore_embedding) {
    //     const double& length_3d = op_cache.length3d = m.mesh_parameters.m_get_length(t);
    //     // adding heuristic decision. If length2 < 4. / 5. * 4. / 5. * m.m_target_l * m.m_target_l always collapse
    //     // enforce heuristic
    //     assert(length_3d < 4. / 5. * m.mesh_parameters.m_quality_threshold);
    // }

    // record the two vertices vids to the operation cache
    op_cache.v1 = my_vid;
    op_cache.v2 = other_vid;
    op_cache.partition_id = my_vattr.partition_id;

    op_cache.constrained_boundary_type = get_constrained_boundary_type(m, t);

    store_merged_seam_data(m, t);
}
bool AdaptiveTessellationEdgeCollapseOperation::check_edge_mergeability(
    const AdaptiveTessellation& m,
    const Tuple& edge) const
{
    // opposing edges are mergeable if only one of them is boundary
    auto mergeable = [&](const Tuple& e) -> bool {
        const Tuple e0 = edge.switch_edge(m);
        const Tuple e1 = e0.switch_vertex(m).switch_edge(m);

        const bool e0_is_boundary = m.is_boundary_edge(e0);
        const bool e1_is_boundary = m.is_boundary_edge(e1);
        if (e0_is_boundary && e1_is_boundary) {
            return false;
        }
        return true;
    };

    // alternate impl that was used for debug
#if defined(LET_TRIMESH_NONMANIFOLD)
    // lazy code that uses VIDs to check other edges
    auto edge_to_vids = [&](const Tuple& e) -> std::array<size_t, 2> {
        const size_t v0 = e.vid(m);
        const size_t v1 = e.switch_vertex().vid(m);
        if (v0 > v1) {
            std::swap(v0, v1);
        }
        return std::array<size_t, 2>{{v0, v1}};
    };
    const auto my_vids = edge_to_vids(edge);
    for (const auto& tri : tris_bounded_by_edge(edge)) {
        Tuple e = tri;
        size_t attempt = 0;
        for (; edge_to_vids(e) != my_vids && attempt < 3; ++attempt) {
            e.switch_edge(m).switch_vertex(m);
        }
        assert(attempt < 3);
        if (!mergeable(e)) {
            return false;
        }
    }
#else

    // check this edge (and potentially the other face across the boundary)
    if (!mergeable(edge)) {
        return false;
    }

    if (const std::optional<Tuple> other_face_opt = edge.switch_face(m);
        other_face_opt.has_value()) {
        if (!mergeable(other_face_opt.value())) {
            return false;
        }
    }
#endif
    return true;
}
bool AdaptiveTessellationEdgeCollapseOperation::check_vertex_mergeability(
    const AdaptiveTessellation& m,
    const Tuple& t) const
{
    const auto& v1_attr = m.get_vertex_attrs(t);
    const auto& v2_attr = m.get_vertex_attrs(t.switch_vertex(m));
    if (v1_attr.fixed || v2_attr.fixed) {
        return false;
    }
    if (m.mesh_parameters.m_bnd_freeze && (v1_attr.boundary_vertex || v2_attr.boundary_vertex)) {
        return false;
    }
    return true;
}

bool AdaptiveTessellationEdgeCollapseOperation::before(AdaptiveTessellation& m, const Tuple& t)
{
    m_op_cache.local() = {};
    if (!wmtk::TriMeshEdgeCollapseOperation::before(m, t)) {
        return false;
    }
    if (!check_vertex_mergeability(m, t)) {
        return false;
    }
    // TODO: currently edge mergeability just checks for double boundaries
    // which is caught by link condition, is there something else?
    // if(!check_edge_mergeability(m, t)) { return false; }

    // record boundary vertex as boudnary_vertex in vertex attribute for accurate collapse
    // after boundary operations

    const size_t my_vid = t.vid(m);
    const Tuple other_tuple = t.switch_vertex(m);
    const size_t other_vid = other_tuple.vid(m);
    auto& my_vattr = m.vertex_attrs[my_vid];
    auto& other_vattr = m.vertex_attrs[other_vid];
    // record if the two vertices of the edge is boundary vertex
    my_vattr.boundary_vertex = m.is_boundary_vertex(t);
    other_vattr.boundary_vertex = m.is_boundary_vertex(other_tuple);


    fill_cache(m, t);

    // make sure that we'll be able to put a vertex in the right positoin
    const ConstrainedBoundaryType cbt = m_op_cache.local().constrained_boundary_type;
    if (cbt == ConstrainedBoundaryType::BothConstrained) {
        return false;
    }
    return true;
}

bool AdaptiveTessellationEdgeCollapseOperation::execute(AdaptiveTessellation& m, const Tuple& t)
{
    OpCache& op_cache = m_op_cache.local();
    assert(m.check_mesh_connectivity_validity());
    return TriMeshEdgeCollapseOperation::execute(m, t);
}
bool AdaptiveTessellationEdgeCollapseOperation::after(AdaptiveTessellation& m)
{
    const auto ret_tup = get_return_tuple_opt();
    if (!bool(ret_tup)) {
        return false;
    }
    OpCache& op_cache = m_op_cache.local();


    return true;
}

auto AdaptiveTessellationEdgeCollapseOperation::assign_new_vertex_attributes(
    AdaptiveTessellation& m,
    const VertexAttributes& attr) const -> VertexAttributes&
{
    return m.get_vertex_attrs(get_return_tuple_opt().value()) = attr;
}

auto AdaptiveTessellationEdgeCollapseOperation::assign_new_vertex_attributes(
    AdaptiveTessellation& m) const -> VertexAttributes&
{
    assert(bool(*this));
    const auto return_edge_tuple = get_return_tuple_opt().value();
    const size_t return_vid = return_edge_tuple.vid(m);
    auto& return_v_attr = m.vertex_attrs[return_vid];

    const OpCache& op_cache = m_op_cache.local();
    const auto& v1_attr = m.vertex_attrs[op_cache.v1];
    const auto& v2_attr = m.vertex_attrs[op_cache.v2];
    switch (op_cache.constrained_boundary_type) {
    case ConstrainedBoundaryType::NoConstraints: {
        return_v_attr.pos = (v1_attr.pos + v2_attr.pos) / 2.0;
        return_v_attr.t = (v1_attr.t + v2_attr.t) / 2.0;
        return_v_attr.partition_id = op_cache.partition_id;
    } break;
    case ConstrainedBoundaryType::TupleSideConstrained: {
        return_v_attr = v1_attr;
    } break;
    case ConstrainedBoundaryType::OtherSideConstrained: {
        return_v_attr = v2_attr;
    } break;
    case ConstrainedBoundaryType::BothConstrained: {
        assert(false); // before should have caught this?
    } break;
    }


    // TODO: figure out if any of htese are redundant / necessary
    return_v_attr.partition_id = op_cache.partition_id;
    // TODO: boundary_vertex is always overwritten / is a cache varible?
    return_v_attr.boundary_vertex = (v1_attr.boundary_vertex || v2_attr.boundary_vertex);
    return_v_attr.fixed = (v1_attr.fixed || v2_attr.fixed);
    return return_v_attr;
}

void AdaptiveTessellationEdgeCollapseOperation::assign_collapsed_edge_attributes(
    AdaptiveTessellation& m,
    const std::optional<Tuple>& new_mirror_vertex_opt) const
{
    auto& tri_connectivity = this->tri_connectivity(m);
    auto nt_opt = new_vertex(m);
    assert(nt_opt.has_value());
    const Tuple& new_vertex_tuple = nt_opt.value();
    const size_t new_vertex_vid = new_vertex_tuple.vid(m);
    auto& op_cache = m_op_cache.local();

    size_t mirror_index = 0;
    if (new_mirror_vertex_opt.has_value()) {
        mirror_index = new_mirror_vertex_opt.value().vid(m);
    }
    //==========
    // for every triangle in the seam-ful one ring nbd lets look for boundary edges
    //==========


    // we cache the edges that do not involve the new vertex lets
    // handle opposing edge pairs update
    auto try_latching_seam_data = [&](size_t v0, size_t v1, const SeamData& data) {
        auto edge_opt = m.tuple_from_edge_vids_opt(v0, v1);

        assert(edge_opt.has_value());
        // edge points to new_vertex
        const Tuple edge = edge_opt.value();

        m.edge_attrs[edge.eid(m)].curve_id = data.curve_id;
        const auto& mopt = data.mirror_edge_vids;
        if (mopt) {
            const auto& mirror_vids = mopt.value();
            auto [m0, m1] = mirror_vids;
            if (m0 == m1) {
                assert(new_mirror_vertex_opt.has_value());
                m0 = mirror_index;
            }
            auto eopt = m.tuple_from_edge_vids_opt(m0, m1);
            assert(eopt.has_value());
            const Tuple mirror = eopt.value();
            assert(mirror.is_valid(m));
            assert(edge.is_valid(m));
            m.set_mirror_edge_data(edge, mirror);
            m.set_mirror_edge_data(mirror, edge);
        }
    };


    // TODO: cache tris that have already been used
    for (const auto& [other_vertex, seam_data] : op_cache.new_vertex_seam_data) {
        try_latching_seam_data(new_vertex_vid, other_vertex, seam_data);
    }
    for (const auto& [edge_vids, seam_data] : op_cache.opposing_edge_seam_data) {
        const auto& [a, b] = edge_vids;
        try_latching_seam_data(a, b, seam_data);
    }
}


bool AdaptiveTessellationPairedEdgeCollapseOperation::input_edge_is_mirror() const
{
    auto& op_cache = m_op_cache.local();
    return op_cache.mirror_edge_tuple_opt.has_value();
}

void AdaptiveTessellationPairedEdgeCollapseOperation::set_input_mirror(
    const AdaptiveTessellation& m,
    const Tuple& t)
{
    auto& op_cache = m_op_cache.local();
    if (m.is_seam_edge(t)) {
        const Tuple mirror = m.get_oriented_mirror_edge(t).switch_vertex(m);
        assert(mirror.is_valid(m));
        op_cache.mirror_edge_tuple_opt = mirror;
    } else {
        op_cache.mirror_edge_tuple_opt = std::nullopt;
    }
}


bool AdaptiveTessellationPairedEdgeCollapseOperation::before(
    AdaptiveTessellation& m,
    const Tuple& t)
{
    auto& op_cache = m_op_cache.local();
    op_cache = {};
    // if (!utils::is_valid_trimesh_topology(m)) {
    //     return false;
    // }


    if (!check_seamed_link_condition(m, t)) {
        return false;
    }
    if (!collapse_edge.before(m, t)) {
        return false;
    }
    if (m.vertex_attrs[t.vid(m)].fixed || m.vertex_attrs[t.switch_vertex(m).vid(m)].fixed) {
        return false;
    }
    set_input_mirror(m, t);


    if (input_edge_is_mirror()) {
        const Tuple& mirror_edge_tuple = op_cache.mirror_edge_tuple_opt.value();
        assert(mirror_edge_tuple.is_valid(m));
        if (!collapse_mirror_edge.before(m, mirror_edge_tuple)) {
            return false;
        }

        // check that if both are constrained they aren't overconstrained
        auto& collapse_edge_cache = collapse_edge.m_op_cache.local();
        auto& mirror_edge_cache = collapse_mirror_edge.m_op_cache.local();
        auto& a = collapse_edge_cache.constrained_boundary_type;
        auto& b = mirror_edge_cache.constrained_boundary_type;

        const auto merged = AdaptiveTessellationEdgeCollapseOperation::merge(a, b);
        if (merged ==
            AdaptiveTessellationEdgeCollapseOperation::ConstrainedBoundaryType::BothConstrained) {
            return false;
        }
        a = merged;
        b = merged;
    }

    store_boundary_data(m, t);

    return true;
}

bool AdaptiveTessellationPairedEdgeCollapseOperation::execute(
    AdaptiveTessellation& m,
    const Tuple& t)
{
    auto& collapse_edge_cache = collapse_edge.m_op_cache.local();
    auto& mirror_edge_cache = collapse_mirror_edge.m_op_cache.local();
    auto& op_cache = m_op_cache.local();


    if (!collapse_edge.execute(m, t)) {
        return false;
    }


    // if we have a mirror edge we need to
    if (input_edge_is_mirror()) {
        const Tuple& mirror_edge_tuple = op_cache.mirror_edge_tuple_opt.value();
        if (!collapse_mirror_edge.execute(m, mirror_edge_tuple)) {
            return false;
        }
    }


    collapse_edge.assign_collapsed_edge_attributes(m, collapse_mirror_edge.new_vertex(m));


    if (input_edge_is_mirror()) {
        collapse_mirror_edge.assign_collapsed_edge_attributes(m, collapse_edge.new_vertex(m));
    }
    return true;
}


bool AdaptiveTessellationPairedEdgeCollapseOperation::after(AdaptiveTessellation& m)
{
    static std::atomic_int cnt = 0;
    auto& op_cache = m_op_cache.local();
    assert(bool(*this));

    Tuple return_tuple = collapse_edge.get_return_tuple_opt().value();
    if (!collapse_edge.after(m)) {
        return false;
    }
    if (input_edge_is_mirror()) {
        if (!collapse_mirror_edge.after(m)) {
            return false;
        }
    }

    const auto& new_vertex_attr = collapse_edge.assign_new_vertex_attributes(m);

    if (input_edge_is_mirror()) {
        // const Tuple& mirror_edge_tuple = collapse_mirror_edge.get_return_tuple_opt().value();
        auto& mirror_vertex_attr = collapse_mirror_edge.assign_new_vertex_attributes(m);

        if (collapse_edge.m_op_cache.local().constrained_boundary_type ==
            AdaptiveTessellationEdgeCollapseOperation::ConstrainedBoundaryType::NoConstraints) {
            mirror_vertex_attr.pos_world = new_vertex_attr.pos_world;
        }
    }
#if defined(_DEBUG)

    if (!utils::is_valid_trimesh_topology(m)) {
        spdlog::info("PairedCollapse invalid before building topology");
        return false;
    }
#endif
    rebuild_boundary_data(m);
#if defined(_DEBUG)
    {
        bool valid = true;
        m.for_each_edge([&](const Tuple& edge) {
            const auto mtup_opt = m.face_attrs[edge.fid(m)].mirror_edges[edge.local_eid(m)];
            if (mtup_opt) {
                if (!mtup_opt->is_valid(m)) {
                    spdlog::warn(
                        "Edge {} has a Mirror edge {} that is invalid",
                        edge.info(),
                        mtup_opt->info());
                    valid = false;
                } else {
                    const auto omtup_opt =
                        m.face_attrs[mtup_opt->fid(m)].mirror_edges[mtup_opt->local_eid(m)];
                    if (!bool(omtup_opt)) {
                        spdlog::warn(
                            "Edge {} has a Mirror edge {} that does not have a mirror",
                            edge.info(),
                            mtup_opt->info());
                        valid = false;
                    } else if (!omtup_opt->is_valid(m)) {
                        spdlog::warn(
                            "Edge {} has a Mirror edge {} that has an invalid mirror {}",
                            edge.info(),
                            mtup_opt->info(),
                            omtup_opt->info());
                    }
                }
            }
        });
    }
    if (!utils::is_valid_trimesh_topology(m)) {
        spdlog::info("PairedCollapse invalid before building topology");
        return false;
    }
#endif

    std::vector<size_t> all_mirrors = m.get_all_mirror_vids(return_tuple);

    std::vector<Tuple> one_ring;
    for (const size_t ret_vid : all_mirrors) {
        const Tuple vtup = m.tuple_from_vertex(ret_vid);
        auto a = m.get_one_ring_tris_for_vertex(vtup);
        one_ring.insert(one_ring.end(), a.begin(), a.end());
    }

    auto mod_tups = modified_triangles(m);
    if (!m.update_energy_cache(mod_tups)) return false;
    // check invariants here since get_area_accuracy_error_per_face requires valid triangle
    if (!m.invariants(one_ring)) return false;
    if (!m.mesh_parameters.m_ignore_embedding) {
        // check acceptance after the energy cache update
        if (m.scheduling_accept_for_collapse(
                modified_triangles(m),
                m.mesh_parameters.m_accuracy_threshold)) {
            return true;
        } else {
            return false;
        }
    }


    // m.write_obj_displaced(
    //     m.mesh_parameters.m_output_folder + fmt::format("/collapse_{:04d}.obj", cnt));
    cnt++;
    // if (!utils::is_valid_trimesh_topology(m)) {
    //     return false;
    // }
    return true;
}

AdaptiveTessellationPairedEdgeCollapseOperation::operator bool() const
{
    return operation_success_T(collapse_edge, collapse_mirror_edge, input_edge_is_mirror());
}
void AdaptiveTessellationPairedEdgeCollapseOperation::mark_failed()
{
    collapse_edge.mark_failed();
    collapse_mirror_edge.mark_failed();
}

auto AdaptiveTessellationPairedEdgeCollapseOperation::get_mirror_edge_tuple_opt() const
    -> std::optional<Tuple>
{
    // note that curveid are handled by the individual operations
    auto& op_cache = m_op_cache.local();

    return op_cache.mirror_edge_tuple_opt;
}
double AdaptiveTessellationPairedEdgeCollapseOperation::priority(
    const TriMesh& m,
    const Tuple& edge) const
{
    const AdaptiveTessellation& at = static_cast<const AdaptiveTessellation&>(m);

    double priority = 0.;
    if (at.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
        // priority already scaled by 2d edge length
        return -at.get_cached_area_accuracy_error_per_edge(edge) * at.get_length2d(edge);
    } else if (at.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::TRI_QUADRICS) {
        // error is not scaled by 2d edge length
        return -at.get_quadrics_area_accuracy_error_for_split(edge) * at.get_length2d(edge);
    } else
        return -at.mesh_parameters.m_get_length(edge);
}

void AdaptiveTessellation::collapse_all_edges()
{
    // collapse is not define for EDGE_QUADRATURE
    // collapse in AREA_QUADRATURE uses 3d edge length

    for (auto f : get_faces()) assert(!is_inverted(f));
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    auto collect_tuples = tbb::concurrent_vector<Tuple>();

    for_each_edge([&](auto& tup) { collect_tuples.emplace_back(tup); });
    collect_all_ops.reserve(collect_tuples.size());
    for (auto& t : collect_tuples) collect_all_ops.emplace_back("edge_collapse", t);
    wmtk::logger().info("=======collapse==========");
    wmtk::logger().info("size for edges to be collapse is {}", collect_all_ops.size());
    auto setup_and_execute = [&](auto executor) {
        addPairedCustomOps(executor);
        executor.renew_neighbor_tuples = renew;
        executor.priority = [&](auto& m, auto o, auto& e) {
            if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
                // priority already scaled by 2d edge length
                return -m.get_length2d(e);
            } else if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::TRI_QUADRICS) {
                // error is not scaled by 2d edge length
                return -m.get_quadrics_area_accuracy_error_for_split(e) * m.get_length2d(e);
            } else
                return -m.mesh_parameters.m_get_length(e);
            return executor.new_edit_operation_maps[o]->priority(m, e);
        };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [weight, op, tup] = ele;
            double error = m.get_length2d(tup);
            if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
                // priority already scaled by 2d edge length
                error = m.get_length2d(tup);
            } else if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::TRI_QUADRICS) {
                // error is not scaled by 2d edge length
                error = m.get_quadrics_area_accuracy_error_for_split(tup) * m.get_length2d(tup);
            } else
                error = m.mesh_parameters.m_get_length(tup);
            if (error != -weight) {
                // wmtk::logger().info("outdated weight in queue");
                // wmtk::logger().info("energy is {}, weight is {}", energy, weight);
                return false;
            }
            if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::LINEAR3D ||
                m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::LINEAR2D) {
                if (error > (4. / 5. * m.mesh_parameters.m_quality_threshold)) return false;
            }
            return true;
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<AdaptiveTessellation, ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<AdaptiveTessellation, ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }
}


void AdaptiveTessellationPairedEdgeCollapseOperation::store_boundary_data(
    const AdaptiveTessellation& m,
    const Tuple& t)
{
    auto& collapse_op_cache = collapse_edge.m_op_cache.local();
    auto& mirror_collapse_op_cache = collapse_mirror_edge.m_op_cache.local();
    OpCache& op_cache = m_op_cache.local();
}
void AdaptiveTessellationPairedEdgeCollapseOperation::rebuild_boundary_data(AdaptiveTessellation& m)
{
    auto& collapse_op_cache = collapse_edge.m_op_cache.local();
    OpCache& op_cache = m_op_cache.local();

    assert(bool(collapse_edge));
    const TriMeshTuple return_tuple = collapse_edge.get_return_tuple_opt().value();

    // stich mirror data
    if (input_edge_is_mirror()) {
        const int ov1 = collapse_op_cache.v1;
        const int ov2 = collapse_op_cache.v2;
        auto& mirror_collapse_op_cache = collapse_mirror_edge.m_op_cache.local();
        const int mv1 = mirror_collapse_op_cache.v1;
        const int mv2 = mirror_collapse_op_cache.v2;


        // ov1 ~ mv2, mv2 ~ ov1
    }
}
auto AdaptiveTessellationPairedEdgeCollapseOperation::modified_triangles(const TriMesh& m) const
    -> std::vector<Tuple>
{
    const auto& at = static_cast<const AdaptiveTessellation&>(m);
    if (!bool(*this)) {
        return {};
    }

    const Tuple new_v = collapse_edge.get_return_tuple_opt().value();

    if (at.is_seam_vertex(new_v)) {
        for (const auto& vert : at.get_all_mirror_vertices(new_v)) {
        }
    }
    auto r = at.get_one_ring_tris_accross_seams_for_vertex(new_v);
    return r;
    // return collapse_edge.modified_triangles(m);
}
auto AdaptiveTessellationEdgeCollapseOperation::modified_triangles(const TriMesh& m) const
    -> std::vector<Tuple>
{
    return TriMeshEdgeCollapseOperation::modified_triangles(m);
    // const auto& at  = static_cast<const AdaptiveTessellation&>(m);
    // if (!bool(*this)) {
    //     return {};
    // }

    // const Tuple new_v = get_return_tuple_opt().value();

    // return at.get_one_ring_tris_accross_seams_for_vertex(new_v);
}
