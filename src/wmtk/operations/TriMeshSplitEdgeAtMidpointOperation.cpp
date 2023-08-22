#include "TriMeshSplitEdgeAtMidpointOperation.hpp"

#include <wmtk/TriMesh.hpp>
#include "TriMeshSplitEdgeOperation.hpp"

namespace wmtk {
TriMeshSplitEdgeAtMidpointOperation::TriMeshSplitEdgeAtMidpointOperation(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<TriMeshSplitEdgeAtMidpointOperation>& settings)
    : Operation(m)
    , m_input_tuple{t}
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_settings{settings}
{
    p0 = m_pos_accessor.vector_attribute(m_input_tuple);
    p1 = m_pos_accessor.vector_attribute(m_mesh.switch_vertex(m_input_tuple));
}
std::string TriMeshSplitEdgeAtMidpointOperation::name() const
{
    return "tri_mesh_split_edge_at_midpoint";
}
Tuple TriMeshSplitEdgeAtMidpointOperation::return_tuple() const
{
    return m_output_tuple;
}
bool TriMeshSplitEdgeAtMidpointOperation::before() const
{
    if (m_mesh.is_outdated(m_input_tuple) || !m_mesh.is_valid(m_input_tuple)) {
        return false;
    }

    const double l_squared = (p1 - p0).squaredNorm();
    return l_squared > m_settings.min_squared_length;
}
bool TriMeshSplitEdgeAtMidpointOperation::execute()
{
    {
        OperationSettings<TriMeshSplitEdgeOperation> op_settings;
        op_settings.split_boundary_edges = m_settings.split_boundary_edges;

        TriMeshSplitEdgeOperation split_op(m_mesh, m_input_tuple, op_settings);
        if (!split_op()) {
            return false;
        }
        m_output_tuple = split_op.return_tuple();
    }

    m_pos_accessor.vector_attribute(m_output_tuple) = 0.5 * (p0 + p1);

    return true;
}
} // namespace wmtk