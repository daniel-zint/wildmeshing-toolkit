#include "EdgeSplitAtMidpoint.hpp"
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/MinEdgeLengthInvariant.hpp>
#include "EdgeSplit.hpp"

namespace wmtk::operations {


void OperationSettings<tri_mesh::EdgeSplitAtMidpoint>::initialize_invariants(const TriMesh& m)
{
    split_settings.initialize_invariants(m);
    split_settings.invariants.add(
        std::make_shared<MinEdgeLengthInvariant>(m, position, min_squared_length));
}

bool OperationSettings<tri_mesh::EdgeSplitAtMidpoint>::are_invariants_initialized() const
{
    return split_settings.are_invariants_initialized() &&
           find_invariants_in_collection_by_type<MinEdgeLengthInvariant>(split_settings.invariants);
}
namespace tri_mesh {
EdgeSplitAtMidpoint::EdgeSplitAtMidpoint(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<EdgeSplitAtMidpoint>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.split_settings.invariants, t)
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_tag_accessor{m.create_accessor(settings.tag)}
    , m_settings{settings}
{
    p0 = m_pos_accessor.vector_attribute(input_tuple());
    p1 = m_pos_accessor.vector_attribute(mesh().switch_vertex(input_tuple()));
    t0 = t1 = 0;
    if (settings.for_extraction) {
        // t0 = (m_tag_accessor.vector_attribute(input_tuple()))(0);
        // t1 = (m_tag_accessor.vector_attribute(mesh().switch_vertex(input_tuple())))(0);
    }
}
std::string EdgeSplitAtMidpoint::name() const
{
    return "tri_mesh_split_edge_at_midpoint";
}
Tuple EdgeSplitAtMidpoint::return_tuple() const
{
    return m_output_tuple;
}
bool EdgeSplitAtMidpoint::before() const
{
    if (m_settings.for_extraction)
        return TupleOperation::before() &&
               (t0 != Iso_Vertex_Type::INPUT || t1 != Iso_Vertex_Type::INPUT);
    else
        return TupleOperation::before();
}
bool EdgeSplitAtMidpoint::execute()
{
    {
        EdgeSplit split_op(mesh(), input_tuple(), m_settings.split_settings);
        if (!split_op()) {
            return false;
        }
        m_output_tuple = split_op.return_tuple();
    }

    m_pos_accessor.vector_attribute(m_output_tuple) = 0.5 * (p0 + p1);
    if (m_settings.for_extraction) {
        // if (t0 == Iso_Vertex_Type::OFFSET && t1 == Iso_Vertex_Type::OFFSET)
        //     (m_tag_accessor.vector_attribute(m_output_tuple))(0) = Iso_Vertex_Type::OFFSET;
        // else
        //     (m_tag_accessor.vector_attribute(m_output_tuple))(0) = Iso_Vertex_Type::SCALFFOLD;
    }

    return true;
}
} // namespace tri_mesh
} // namespace wmtk::operations
