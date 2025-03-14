#include "UpdateVertexMultiMeshMapHash.hpp"
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>

namespace wmtk::operations::utils {
void update_vertex_operation_hashes(Mesh& m, const Tuple& vertex, Accessor<long>& hash_accessor)
{
    const PrimitiveType pt = m.top_simplex_type();
    const SimplicialComplex star = SimplicialComplex::closed_star(m, Simplex::vertex(vertex));
    std::vector<Tuple> tuples_to_update;
    switch (pt) {
    case PrimitiveType::Vertex: {
        const auto star_vertices = star.get_vertices();
        tuples_to_update.reserve(star_vertices.size());
        for (const Simplex& s : star_vertices) {
            tuples_to_update.emplace_back(s.tuple());
        }
        break;
    }
    case PrimitiveType::Edge: {
        const auto star_edges = star.get_edges();
        tuples_to_update.reserve(star_edges.size());
        for (const Simplex& s : star_edges) {
            tuples_to_update.emplace_back(s.tuple());
        }
        break;
    }
    case PrimitiveType::Face: {
        const auto star_faces = star.get_faces();
        tuples_to_update.reserve(star_faces.size());
        for (const Simplex& s : star_faces) {
            tuples_to_update.emplace_back(s.tuple());
        }
        break;
    }
    case PrimitiveType::Tetrahedron: {
        const auto star_tets = star.get_tetrahedra();
        tuples_to_update.reserve(star_tets.size());
        for (const Simplex& s : star_tets) {
            tuples_to_update.emplace_back(s.tuple());
        }
        break;
    }
    }


    m.update_cell_hashes(tuples_to_update, hash_accessor);

    // need to get a star with new hash, otherwise cannot get_simplices()
    auto vertex_new_hash = m.resurrect_tuple(vertex, hash_accessor);
    const SimplicialComplex star_new_hash =
        SimplicialComplex::closed_star(m, Simplex::vertex(vertex_new_hash));
    update_vertex_operation_multimesh_map_hash(m, star_new_hash, hash_accessor);
}

void update_vertex_operation_multimesh_map_hash(
    Mesh& m,
    const SimplicialComplex& vertex_closed_star,
    Accessor<long>& parent_hash_accessor)
{
    auto& mm_manager = m.m_multi_mesh_manager;

    for (auto& child_data : mm_manager.children()) {
        auto& child_mesh = *child_data.mesh;
        auto child_mesh_pt = child_mesh.top_simplex_type();
        auto maps = mm_manager.get_map_accessors(m, child_data);
        auto& [parent_to_child_accessor, child_to_parent_accessor] = maps;

        const auto parent_simplices_to_update = vertex_closed_star.get_simplices(child_mesh_pt);
        for (auto& s : parent_simplices_to_update) {
            auto s_tuple = s.tuple();
            auto [parent_tuple, child_tuple] =
                wmtk::multimesh::utils::read_tuple_map_attribute(parent_to_child_accessor, s_tuple);

            if (parent_tuple.is_null()) continue;

            // resurrect the parent tuple
            parent_tuple = m.resurrect_tuple(parent_tuple, parent_hash_accessor);

            // write back to map
            wmtk::multimesh::utils::symmetric_write_tuple_map_attributes(
                parent_to_child_accessor,
                child_to_parent_accessor,
                parent_tuple,
                child_tuple);
        }
    }
}

} // namespace wmtk::operations::utils