#include "BuildOffsetPost.hpp"
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/MinEdgeLengthInvariant.hpp>
#include "EdgeSplit.hpp"

namespace wmtk::operations {

namespace tri_mesh {
BuildOffsetPost::BuildOffsetPost(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<BuildOffsetPost>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.split_settings.invariants, t)
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_tag_accessor{m.create_accessor(settings.tag)}
    , m_settings{settings}
{
    p0 = m_pos_accessor.vector_attribute(input_tuple());
    p1 = m_pos_accessor.vector_attribute(mesh().switch_vertex(input_tuple()));
    p2 = m_pos_accessor.vector_attribute(mesh().switch_vertex(mesh().switch_edge(input_tuple())));

    t0 = (m_tag_accessor.vector_attribute(input_tuple()))(0);
    t1 = (m_tag_accessor.vector_attribute(mesh().switch_vertex(input_tuple())))(0);
    t2 = (m_tag_accessor.vector_attribute(mesh().switch_vertex(mesh().switch_edge(input_tuple()))))(
        0);
}
std::string BuildOffsetPost::name() const
{
    return "build_offset_post";
}
Tuple BuildOffsetPost::return_tuple() const
{
    return m_output_tuple;
}
bool BuildOffsetPost::before() const
{
    bool need_split = false;

    need_split = (t0 == Iso_Vertex_Type::OFFSET) && (t1 == Iso_Vertex_Type::OFFSET) &&
                 (t2 == Iso_Vertex_Type::OFFSET);

    return need_split;
}
bool BuildOffsetPost::execute()
{
    {
        EdgeSplit split_op1(mesh(), input_tuple(), m_settings.split_settings);
        if (!split_op1()) {
            return false;
        }
        EdgeSplit split_op2(mesh(), split_op1.return_tuple(), m_settings.split_settings);
        if (!split_op2()) {
            return false;
        }
        m_output_tuple = split_op2.return_tuple();
        EdgeCollapse collapse_op(mesh(), input_tuple(), m_settings.collapse_settings);
        if (!collapse_op()) {
            return false;
        }
    }

    m_pos_accessor.vector_attribute(m_output_tuple) = (p0 + p1 + p2) / 3.0;
    m_tag_accessor.vector_attribute(m_output_tuple)(0) =
        Iso_Vertex_Type::OFFSET; // tag the vertex as a offset

    return true;
}
} // namespace tri_mesh
} // namespace wmtk::operations
