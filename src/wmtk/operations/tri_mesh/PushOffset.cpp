#include "PushOffset.hpp"

#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include "VertexSmooth.hpp"

namespace wmtk::operations::tri_mesh {
PushOffset::PushOffset(Mesh& m, const Tuple& t, const OperationSettings<PushOffset>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.invariants, t)
    , m_pos_accessor(m.create_accessor<double>(settings.position))
    , m_tag_accessor(m.create_accessor<long>(settings.tag))
    , m_settings{settings}
{}

std::string PushOffset::name() const
{
    return "tri_mesh_vertex_smooth";
}

const Tuple& PushOffset::return_tuple() const
{
    return m_output_tuple;
}

bool PushOffset::before() const
{
    if (!mesh().is_valid_slow(input_tuple())) {
        return false;
    }
    if (!m_settings.smooth_boundary && mesh().is_boundary_vertex(input_tuple())) {
        return false;
    }
    if ((m_tag_accessor.vector_attribute(input_tuple()))(0) != OFFSET) {
        return false;
    }
    return true;
}

bool PushOffset::execute()
{
    const std::vector<Simplex> one_ring = SimplicialComplex::vertex_one_ring(mesh(), input_tuple());
    auto p_mid = m_pos_accessor.vector_attribute(input_tuple());
    p_mid = Eigen::Vector3d::Zero();
    int debug_cnt_offset = 0;
    int debug_cnt_neighbour = 0;
    // I'm not sure if it is okay to use Tuple as an element
    std::vector<Tuple> neighbour_input_vertices;
    for (const Simplex& s : one_ring) {
        if ((m_tag_accessor.vector_attribute(s.tuple()))(0) == OFFSET) {
            debug_cnt_offset++;
            p_mid += m_pos_accessor.vector_attribute(s.tuple());
        } else if ((m_tag_accessor.vector_attribute(s.tuple()))(0) == INPUT) {
            debug_cnt_neighbour++;
            neighbour_input_vertices.push_back(s.tuple());
        }
    }
    assert(debug_cnt_offset == 2);
    assert(debug_cnt_neighbour > 1);
    p_mid /= 2.0;

    Eigen::Vector3d origin = computeOriginOnInput(input_tuple(), neighbour_input_vertices);
    Eigen::Vector3d n = (p_mid - origin).normalized();

    m_pos_accessor.vector_attribute(input_tuple()) = origin + n * m_settings.distance;
    // check inversion cases
    // ...
    return true;
}

Eigen::Vector3d PushOffset::computeOriginOnInput(
    const Tuple& t,
    const std::vector<Tuple> neighbours)
{
    Eigen::Vector3d tP = m_pos_accessor.vector_attribute(t);
    Eigen::Vector3d ret;
    double min_distance = std::numeric_limits<double>::max();
    for (auto& neighbour : neighbours) {
        const std::vector<Simplex> one_ring = SimplicialComplex::vertex_one_ring(mesh(), neighbour);
        for (const Simplex& s : one_ring) {
            if ((m_tag_accessor.vector_attribute(s.tuple()))(0) == INPUT) {
                Eigen::Vector3d closeP = getClosestPointFromEdge(t, neighbour, s.tuple());
                if ((closeP - tP).norm() < min_distance) {
                    min_distance = (closeP - tP).norm();
                    ret = closeP;
                }
            }
        }
    }
    return ret;
}

Eigen::Vector3d
PushOffset::getClosestPointFromEdge(const Tuple& t0, const Tuple& et0, const Tuple& et1)
{
    auto p0 = m_pos_accessor.vector_attribute(et0);
    auto p1 = m_pos_accessor.vector_attribute(et1);
    auto p2 = m_pos_accessor.vector_attribute(t0);
    Eigen::Vector3d v0, v1, vp, n;
    v0.x() = p0.x();
    v0.y() = p0.y();
    v1.x() = p1.x();
    v1.y() = p1.y();
    vp.x() = p2.x();
    vp.y() = p2.y();

    if ((vp - v0).dot(v1 - v0) <= 0) {
        return v0;
    } else if ((vp - v1).dot(v0 - v1) <= 0) {
        return v1;
    }

    n = (v1 - v0).normalized();
    double n0, n1, t, m0, m1;
    n0 = n(0);
    n1 = n(1);
    m0 = (v0 - vp).x();
    m1 = (v0 - vp).y();
    t = -(m0 * n0 + m1 * n1) / (n1 * n1 + n0 * n0);
    return v0 + n * t;
}


} // namespace wmtk::operations::tri_mesh
