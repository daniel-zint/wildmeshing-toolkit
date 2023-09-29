#include "embedding.hpp"
#include <wmtk/io/MeshReader.hpp>

#include <igl/edge_topology.h>
#include <igl/read_triangle_mesh.h>
#include <spdlog/spdlog.h>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/EdgeMeshReader.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include "internal/Embedding.hpp"
#include "internal/EmbeddingOptions.hpp"

namespace wmtk {
namespace components {
// this data is the one Prototype displayed

void embedding(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    EmbeddingOptions options = j.get<EmbeddingOptions>();

    Eigen::MatrixXd vertices_, vertices;
    Eigen::Matrix<long, -1, -1> edges;

    // EdgeMeshReader reader();
    EdgeMesh mesh;
    EdgeMeshReader reader(files[options.input_file], EdgeMeshReader::OBJ);
    reader.read(edges, vertices_);
    vertices.resize(vertices_.rows(), 2);

    // assume we have the data
    // one thing should keep in mind, in 2D case, igl::triangulate can
    // only operate vertices with type Eigen::MatrixXd(n x 2)
    // it will crash if the vertices have the thrid dimension value.
    // size convert
    for (long i = 0; i < vertices_.rows(); ++i) {
        // spdlog::info("{} {}", vertices_(i, 0), vertices_(i, 1));
        vertices(i, 0) = vertices_(i, 0);
        vertices(i, 1) = vertices_(i, 1);
    }

    spdlog::info("edges:{} vertices:{} ", edges.rows(), vertices.rows());
    Embedding embedding(edges, vertices, options);
    embedding.process();
    spdlog::info("faces:{} vertices:{} ", embedding.m_faces.rows(), embedding.m_vertices.rows());

    // these are output attributes
    // embeddedRemeshing2D.m_vertex_tags; // Flags
    // embeddedRemeshing2D.m_vertices; // Vertices
    // embeddedRemeshing2D.m_faces; // Faces

    // output
    {
        const std::filesystem::path cache_dir = "cache";
        const std::filesystem::path cached_mesh_file = cache_dir / (options.output + ".hdf5");
        // write to the cache
        HDF5Writer writer(cached_mesh_file);

        TriMesh tri_mesh;
        tri_mesh.initialize(embedding.m_faces);
        Eigen::Matrix<long, -1, -1> tags(embedding.m_vertices.rows(), 1);
        for (long i = 0; i < embedding.m_vertices.rows(); i++) {
            tags(i, 0) = embedding.m_vertex_tags[i];
        }

        mesh_utils::set_matrix_attribute(tags, "m_tags", PrimitiveType::Vertex, tri_mesh);
        tri_mesh.serialize(writer);

        files[options.output] = cached_mesh_file;
    }
}
} // namespace components
} // namespace wmtk