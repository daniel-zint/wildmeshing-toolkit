#include "local_switch_tuple.hpp"
#include <stdexcept>
#include <wmtk/autogen/utils/TupleInspector.hpp>
#include "autogenerated_tables.hpp"
#include "local_id_table_offset.hpp"


namespace wmtk::autogen::tri_mesh {
Tuple local_switch_tuple(const Tuple& tuple, PrimitiveType pt)
{
    using namespace utils;
    const long offset = local_id_table_offset(tuple);
    switch (pt) {
    case PrimitiveType::Vertex:
        return Tuple(
            auto_2d_table_vertex[offset][0],
            auto_2d_table_vertex[offset][1],
            TupleInspector::local_fid(tuple),
            TupleInspector::global_cid(tuple),
            TupleInspector::hash(tuple));

    case PrimitiveType::Edge:
        return Tuple(
            auto_2d_table_edge[offset][0],
            auto_2d_table_edge[offset][1],
            TupleInspector::local_fid(tuple),
            TupleInspector::global_cid(tuple),
            TupleInspector::hash(tuple));

    case PrimitiveType::Face:
    case PrimitiveType::Tetrahedron:
    default: throw std::runtime_error("Tuple switch: Invalid primitive type"); break;
    }
    return Tuple();
}
} // namespace wmtk::autogen::tri_mesh
