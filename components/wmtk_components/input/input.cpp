#include "input.hpp"

#include <igl/edges.h>
#include <igl/read_triangle_mesh.h>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/EdgeMeshReader.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include "internal/InputOptions.hpp"

namespace wmtk {
namespace components {

void input(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    InputOptions options = j.get<InputOptions>();

    if (!std::filesystem::exists(options.file)) {
        throw std::runtime_error(std::string("file") + options.file.string() + " not found");
    }

    switch (options.cell_dimension) {
    case 0: {
        // point-cloud
        PointMesh mesh;
        if (options.file.extension() == ".hdf5") {
            MeshReader reader(options.file);
            reader.read(mesh);
        } else if (options.file.extension() == ".off" || options.file.extension() == ".obj") {
            Eigen::MatrixXd V;
            Eigen::Matrix<long, -1, -1> F;
            igl::read_triangle_mesh(options.file.string(), V, F);

            mesh.initialize(V.rows());

            mesh_utils::set_matrix_attribute(V, "position", PrimitiveType::Vertex, mesh);

        } else {
            throw std::runtime_error(std::string("Unknown file type: ") + options.file.string());
        }

        const std::filesystem::path cache_dir = "cache";
        std::filesystem::create_directory(cache_dir);

        const std::filesystem::path cached_mesh_file = cache_dir / (options.name + ".hdf5");

        HDF5Writer writer(cached_mesh_file);
        mesh.serialize(writer);

        files[options.name] = cached_mesh_file;
        break;
    }
    case 1: {
        // edge mesh
        spdlog::warn("this edge mesh is old version, need to update according to the latest "
                     "EdgeMesh branch!");
        // for debugging
        // spdlog::info("file_name:{}", options.file.string());
        EdgeMesh mesh;
        if (options.file.extension() == ".hdf5") {
            MeshReader reader(options.file);
            reader.read(mesh);
        } else if (options.file.extension() == ".obj" || options.file.extension() == ".off") {
            Eigen::MatrixXd V;
            Eigen::Matrix<long, -1, -1> E;
            // read E
            // assume only have l and v
            EdgeMeshReader::file_type type;
            if (options.file.extension() == ".obj") {
                type = EdgeMeshReader::OBJ;
            } else if (options.file.extension() == ".off") {
                type = EdgeMeshReader::OFF;
            }
            // EdgeMeshReader reader(, );
            EdgeMeshReader reader(options.file.string(), EdgeMeshReader::file_type::OBJ);
            reader.read(E, V);
            mesh.initialize(E);
            // for debugging
            // spdlog::info("E:{} V:{}", E.rows(), V.rows());

            Eigen::Matrix<long, -1, -1> vertices_tags;

            // I thought this block could be removed, since we will add tags in embedding part.
            vertices_tags.resize(V.rows(), 1);
            vertices_tags.setZero();
            mesh_utils::set_matrix_attribute(
                vertices_tags,
                "vertices_tags",
                PrimitiveType::Vertex,
                mesh);

            mesh_utils::set_matrix_attribute(V, "position", PrimitiveType::Vertex, mesh);
        } else {
            throw std::runtime_error(std::string("Unknown file type: ") + options.file.string());
        }

        const std::filesystem::path cache_dir = "cache";
        std::filesystem::create_directory(cache_dir);

        const std::filesystem::path cached_mesh_file = cache_dir / (options.name + ".hdf5");

        HDF5Writer writer(cached_mesh_file);
        mesh.serialize(writer);

        files["file"] = options.file;
        files[options.name] = cached_mesh_file;
        break;
    }
    case 2: {
        // triangle mesh
        TriMesh mesh;
        if (options.file.extension() == ".hdf5") {
            MeshReader reader(options.file);
            reader.read(mesh);
        } else if (options.file.extension() == ".off" || options.file.extension() == ".obj") {
            Eigen::MatrixXd V;
            Eigen::Matrix<long, -1, -1> F;
            igl::read_triangle_mesh(options.file.string(), V, F);

            mesh.initialize(F);

            mesh_utils::set_matrix_attribute(V, "position", PrimitiveType::Vertex, mesh);

        } else {
            throw std::runtime_error(std::string("Unknown file type: ") + options.file.string());
        }

        const std::filesystem::path cache_dir = "cache";
        std::filesystem::create_directory(cache_dir);

        const std::filesystem::path cached_mesh_file = cache_dir / (options.name + ".hdf5");

        HDF5Writer writer(cached_mesh_file);
        mesh.serialize(writer);

        files[options.name] = cached_mesh_file;
        break;
    }
    case 3:
        // tetrahedra mesh
        assert(false);
        break;
    default: assert(false); break;
    }
}
} // namespace components
} // namespace wmtk