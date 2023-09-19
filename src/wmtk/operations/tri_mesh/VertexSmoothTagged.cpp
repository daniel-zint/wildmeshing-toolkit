#include "VertexSmoothTagged.hpp"

#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "VertexSmooth.hpp"

namespace wmtk::operations::tri_mesh {
VertexSmoothTagged::VertexSmoothTagged(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<VertexSmoothTagged>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.smooth_settings.invariants, t)
    , m_pos_accessor{m.create_accessor<double>(settings.smooth_settings.position)}
    , m_tag_accessor{m.create_accessor<long>(settings.smooth_settings.tag)}
    , m_settings{settings}
{}

std::string VertexSmoothTagged::name() const
{
    return "tri_mesh_vertex_tangential_smooth";
}

bool VertexSmoothTagged::before() const
{
    if (!mesh().is_valid_slow(input_tuple())) {
        return false;
    }
    long tag = (m_tag_accessor.vector_attribute(input_tuple()))(0);
    if (tag == OFFSET || tag == INPUT) {
        return false;
    }
    return true;
}

bool VertexSmoothTagged::execute()
{
    const Eigen::Vector3d p = m_pos_accessor.vector_attribute(input_tuple());
    OperationSettings<tri_mesh::VertexSmooth> op_settings;
    tri_mesh::VertexSmooth smooth_op(mesh(), input_tuple(), m_settings.smooth_settings);
    if (!smooth_op()) {
        return false;
    }

    const Tuple tup = smooth_op.return_tuple();


    assert(mesh().is_valid_slow(tup));
    const Eigen::Vector3d g = m_pos_accessor.vector_attribute(tup); // center of gravity

    return true;
}


} // namespace wmtk::operations::tri_mesh
