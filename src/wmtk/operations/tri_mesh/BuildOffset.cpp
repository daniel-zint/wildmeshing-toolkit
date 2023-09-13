#include "BuildOffset.hpp"
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/MinEdgeLengthInvariant.hpp>
#include "EdgeSplit.hpp"

namespace wmtk::operations {

namespace tri_mesh {
BuildOffset::BuildOffset(Mesh& m, const Tuple& t, const OperationSettings<BuildOffset>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.split_settings.invariants, t)
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_tag_accessor{m.create_accessor(settings.tag)}
    , m_settings{settings}
{
    p0 = m_pos_accessor.vector_attribute(input_tuple());
    p1 = m_pos_accessor.vector_attribute(mesh().switch_vertex(input_tuple()));
    t0 = (m_tag_accessor.vector_attribute(input_tuple()))(0);
    t1 = (m_tag_accessor.vector_attribute(mesh().switch_vertex(input_tuple())))(0);
}
std::string BuildOffset::name() const
{
    if (m_settings.pass == PASS_ONE)
        return "build_offset_pass1";
    else if (m_settings.pass == PASS_TWO)
        return "build_offset_pass2";
    else
        printf("ERROR! Didn't set a pass number for build_offset operation. --> BuildOffset.cpp\n");

    abort();
}
Tuple BuildOffset::return_tuple() const
{
    return m_output_tuple;
}
bool BuildOffset::before() const
{
    bool need_split = false;
    if (m_settings.pass == PASS_ONE)
        // pass one, split edge with same "input" tag
        need_split = t0 == Iso_Vertex_Type::INPUT && t1 == Iso_Vertex_Type::INPUT;
    else if (m_settings.pass == PASS_TWO)
        // pass two, split edge with different tag
        need_split = t0 == Iso_Vertex_Type::INPUT || t1 == Iso_Vertex_Type::INPUT;

    return need_split;
}
bool BuildOffset::execute()
{
    {
        EdgeSplit split_op(mesh(), input_tuple(), m_settings.split_settings);
        if (!split_op()) {
            return false;
        }
        m_output_tuple = split_op.return_tuple();
    }

    m_pos_accessor.vector_attribute(m_output_tuple) = 0.5 * (p0 + p1);
    m_tag_accessor.vector_attribute(m_output_tuple)(0) =
        Iso_Vertex_Type::OFFSET; // tag the vertex as a offset

    return true;
}
} // namespace tri_mesh
} // namespace wmtk::operations
