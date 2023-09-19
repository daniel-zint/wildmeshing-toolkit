#include "isosurface_extraction.hpp"

#include <igl/adjacency_list.h>
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

void triangulate(
    Eigen::MatrixXd& V,
    Eigen::MatrixXi& F,
    Eigen::MatrixXi& E,
    std::vector<bool>& Vtags)
{
    // // Eigen::MatrixXd H;
    // // H.resize(1, 2);
    // // H << 10, 0;
    // preprocess
    std::vector<int> markedV;
    std::vector<std::vector<int>> markedE;
    for (int i = 0; i < V.rows(); ++i) {
        markedV.push_back(i);
    }
    for (int i = 0; i < E.rows(); ++i) {
        std::vector<int> new_edge;
        new_edge.push_back(E(i, 0));
        new_edge.push_back(E(i, 1));
        markedE.push_back(new_edge);
    }
    // find boundingbox
    double max_x, max_y;
    Eigen::MatrixXd BoundingBoxV(4, 2);
    Eigen::MatrixXi BoundingBoxE(4, 2);
    BoundingBoxV << max_x, max_y, -max_x, max_y, -max_x, -max_y, max_x, -max_y;
    BoundingBoxE << markedV.size(), markedV.size() + 1, markedV.size() + 1, markedV.size() + 2,
        markedV.size() + 2, markedV.size() + 3, markedV.size() + 3, markedV.size();
    Eigen::MatrixXd tempV;
    Eigen::MatrixXi tempE;
    tempV = V + BoundingBoxV;
    tempE = E + BoundingBoxE;
    // igl::triangle::triangulate(tempV, tempE, H, "a0.1q", V, F);

    // need to connect the topology
    // it would be easy, just check each edge and then move on
    std::vector<std::vector<int>> A;
    igl::adjacency_list(F, A);

    std::vector<std::vector<int>> tempEmarked;
    std::vector<int> tempVmarked;
    for (const auto& e : markedE) {
        int vid0 = e[0], vid1 = e[1];
        Eigen::Vector2<double> p0, p1, dir;
        p0 << V(vid0, 0), V(vid0, 1);
        p1 << V(vid1, 0), V(vid1, 1);
        int cur_vid = vid0;
        tempVmarked.push_back(cur_vid);
        while (cur_vid != vid1) {
            Eigen::Vector2<double> cur_p;
            cur_p << V(cur_vid, 0), V(cur_vid, 1);
            dir = (p1 - cur_p).normalized();
            std::vector<int>& neighbour_v_list = A[cur_vid];
            double value_dir = -1;
            int next_vid = -1;
            for (const auto neighbour_id : neighbour_v_list) {
                Eigen::Vector2<double> p, cur_dir;
                p << V(neighbour_id, 0), V(neighbour_id, 1);
                cur_dir = (p - cur_p).normalized();
                if (value_dir < cur_dir.dot(dir)) {
                    value_dir = cur_dir.dot(dir);
                    next_vid = neighbour_id;
                }
            }
            int vid0_ = cur_vid, vid1_ = next_vid;
            if (vid0_ > vid1_) std::swap(vid0_, vid1_);
            std::vector<int> newedge;
            newedge.push_back(vid0_);
            newedge.push_back(vid1_);
            tempEmarked.push_back(newedge);
            tempVmarked.push_back(next_vid);
            cur_vid = next_vid;
        }
    }
    std::sort(tempVmarked.begin(), tempVmarked.end());
    tempVmarked.erase(std::unique(tempVmarked.begin(), tempVmarked.end()), tempVmarked.end());
    markedE = tempEmarked;
    markedV = tempVmarked;

    Vtags = std::vector<bool>(V.rows(), false);
    for (int i = 0; i < markedV.size(); ++i) {
        Vtags[markedV[i]] = true;
    }
    // this block is used to fix the potential problem
    // for (int i = 0; i < F.rows(); ++i) {
    //     int vid0 = F(i, 0), vid1 = F(i, 1), vid2 = F(i, 2);
    //     if (isMarked(vid0) && isMarked(vid1) && isMarked(vid2)) {
    //         splitface(i);
    //     }
    // }
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
    Eigen::MatrixXi E;
    {
        const std::filesystem::path& file = files[options.input];
        igl::read_triangle_mesh(file.string(), V, F);
    }

    igl::edges(F, E);
    std::vector<bool> Vtags;
    triangulate(V, F, E, Vtags);

    if (options.inflate_abs < 0) {
        if (options.inflate_rel < 0) {
            throw std::runtime_error("Either absolute or relative length must be set!");
        }
        options.inflate_abs = relative_to_absolute_length(V, F, options.inflate_rel);
    }

    IsosurfaceExtraction iso_ex(mesh, V, F, Vtags, options.inflate_abs, options.lock_boundary);
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