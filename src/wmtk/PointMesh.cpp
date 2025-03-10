#include "PointMesh.hpp"

namespace wmtk {
Tuple PointMesh::vertex_tuple_from_id(long id) const
{
    return Tuple(-1, -1, -1, id, get_cell_hash_slow(id));
}

PointMesh::PointMesh()
    : Mesh(0)
{}


PointMesh::PointMesh(long size)
    : PointMesh()
{
    initialize(size);
}

Tuple PointMesh::switch_tuple(const Tuple& tuple, PrimitiveType type) const
{
    throw std::runtime_error("Tuple switch: Invalid primitive type");
}
bool PointMesh::is_ccw(const Tuple&) const
{
    // trivial orientation so nothing can happen
    return true;
}
bool PointMesh::is_boundary(const Tuple& tuple, PrimitiveType pt) const
{
    switch (pt) {
    case PrimitiveType::Vertex: return is_boundary_vertex(tuple);
    case PrimitiveType::Edge:
    case PrimitiveType::Face:
    case PrimitiveType::Tetrahedron:
    case PrimitiveType::HalfEdge:
    default: break;
    }
    throw std::runtime_error(
        "tried to compute hte boundary of an edge mesh for an invalid simplex dimension");
    return false;
    // every point is on the interior as it has no boundary simplices
    return false;
}

bool PointMesh::is_boundary_vertex(const Tuple&) const
{
    // every point is on the interior as it has no boundary simplices
    return false;
}

void PointMesh::initialize(long count)
{
    set_capacities({count});
    reserve_attributes_to_fit();
    Accessor<char> v_flag_accessor = get_flag_accessor(PrimitiveType::Vertex);
    for (long i = 0; i < capacity(PrimitiveType::Vertex); ++i) {
        v_flag_accessor.index_access().scalar_attribute(i) |= 0x1;
    }
}


bool PointMesh::is_valid(const Tuple& tuple, ConstAccessor<long>& hash_accessor) const
{
    if (tuple.is_null()) return false;
    return true;
    return Mesh::is_hash_valid(tuple, hash_accessor);
}

long PointMesh::id(const Tuple& tuple, PrimitiveType type) const
{
    switch (type) {
    case PrimitiveType::Vertex: return tuple.m_global_cid;
    case PrimitiveType::Edge:
    case PrimitiveType::HalfEdge:
    case PrimitiveType::Face:
    case PrimitiveType::Tetrahedron:
    default: throw std::runtime_error("Tuple switch: Invalid primitive type"); break;
    }
}

Tuple PointMesh::tuple_from_id(const PrimitiveType type, const long gid) const
{
    if (type != PrimitiveType::Vertex) {
        throw std::runtime_error("Tuple switch: Invalid primitive type");
    }
    return vertex_tuple_from_id(gid);
}
} // namespace wmtk
