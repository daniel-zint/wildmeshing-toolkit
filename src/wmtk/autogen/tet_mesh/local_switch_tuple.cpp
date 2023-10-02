#include "local_switch_tuple.hpp"
#include <stdexcept>
#include <wmtk/autogen/utils/TupleInspector.hpp>
#include "autogenerated_tables.hpp"
#include "local_id_table_offset.hpp"


namespace wmtk::autogen::tet_mesh {
Tuple local_switch_tuple(const Tuple& tuple, PrimitiveType pt)
{
    using namespace utils;
    const long offset = local_id_table_offset(tuple);


    const long global_cid = TupleInspector::global_cid(tuple);
    const long hash = TupleInspector::hash(tuple);
    switch (pt) {
    case PrimitiveType::Vertex:
        return Tuple(
            auto_3d_table_vertex[offset][0],
            auto_3d_table_vertex[offset][1],
            auto_3d_table_vertex[offset][2],
            global_cid,
            hash);

    case PrimitiveType::Edge:
        return Tuple(
            auto_3d_table_edge[offset][0],
            auto_3d_table_edge[offset][1],
            auto_3d_table_edge[offset][2],
            global_cid,
            hash);
    case PrimitiveType::Face:
        return Tuple(
            auto_3d_table_face[offset][0],
            auto_3d_table_face[offset][1],
            auto_3d_table_face[offset][2],
            global_cid,
            hash);

    case PrimitiveType::Tetrahedron:
    default: throw std::runtime_error("Tuple switch: Invalid primitive type"); break;
    }
    return Tuple();
}
} // namespace wmtk::autogen::tet_mesh
