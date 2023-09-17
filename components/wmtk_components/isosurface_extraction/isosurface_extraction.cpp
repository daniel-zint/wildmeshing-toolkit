#include "isosurface_extraction.hpp"

#include <igl/avg_edge_length.h>
#include <igl/read_triangle_mesh.h>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>

#include "internal/IsosurfaceExtraction.hpp"
#include "internal/IsosurfaceExtractionOptions.hpp"

namespace wmtk {
namespace components {
// compute the length relative to the bounding box diagonal
double relative_to_absolute_length(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const double& inflate_rel)
{
    return igl::avg_edge_length(V, F) * inflate_rel;
}

void isosurface_extraction(
    const nlohmann::json& j,
    std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    IsosurfaceExtractionOptions options = j.get<IsosurfaceExtractionOptions>();

    // input
    TriMesh mesh;
    {
        const std::filesystem::path& file = files[options.input];
        MeshReader reader(file);
        reader.read(mesh);
    }

    // input
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    {
        const std::filesystem::path& file = files[options.input];
        igl::read_triangle_mesh(file.string(), V, F);
    }

    if (options.inflate_abs < 0) {
        if (options.inflate_rel < 0) {
            throw std::runtime_error("Either absolute or relative length must be set!");
        }
        options.inflate_abs = relative_to_absolute_length(V, F, options.inflate_rel);
    }

    IsosurfaceExtraction iso_ex(mesh, V, F, options.inflate_abs, options.lock_boundary);
    iso_ex.process(options.iteration_times);

    // output
    {
        const std::filesystem::path cache_dir = "cache";
        const std::filesystem::path cached_mesh_file = cache_dir / (options.output + ".hdf5");

        HDF5Writer writer(cached_mesh_file);
        mesh.serialize(writer);

        files[options.output] = cached_mesh_file;
    }
}
} // namespace components
} // namespace wmtk