#include "IsosurfaceExtraction.hpp"

#include <igl/edges.h>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/operations/tri_mesh/BuildOffset.hpp>
#include <wmtk/operations/tri_mesh/BuildOffsetPost.hpp>
#include <wmtk/operations/tri_mesh/EdgeCollapseToMidpoint.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplitAtMidpoint.hpp>
#include <wmtk/operations/tri_mesh/EdgeSwap.hpp>
#include <wmtk/operations/tri_mesh/VertexTangentialSmooth.hpp>

namespace wmtk::components::internal {

IsosurfaceExtraction::IsosurfaceExtraction(
    TriMesh& mesh,
    Eigen::MatrixXd& V,
    Eigen::MatrixXi& F,
    const double length,
    const bool lock_boundary)
    : m_mesh{mesh}
    , m_length_min{(4. / 5.) * length}
    , m_length_max{(4. / 3.) * length}
    , m_lock_boundary{lock_boundary}
    , m_scheduler(m_mesh)
{
    using namespace operations;
    // triangulate
    Eigen::MatrixXi E;
    igl::edges(F, E);
    // more things todo...

    m_position_handle = m_mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    m_tag_handle = m_mesh.get_attribute_handle<int>("tag", PrimitiveType::Vertex);
    // offset computation
    {
        // BuildOffset Operation, has two passes
        // BuildOffsetPost Operation, an additional pass for resolution
        OperationSettings<tri_mesh::BuildOffset> buildsettings_pass1;
        buildsettings_pass1.position = m_position_handle;
        buildsettings_pass1.tag = m_tag_handle;
        buildsettings_pass1.pass = tri_mesh::BuildOffset::PASS_ONE;
        buildsettings_pass1.split_settings.split_boundary_edges = !m_lock_boundary;
        m_scheduler.add_operation_type<tri_mesh::BuildOffset>(
            "buildoffset_pass1",
            buildsettings_pass1);

        OperationSettings<tri_mesh::BuildOffset> buildsettings_pass2;
        buildsettings_pass2.position = m_position_handle;
        buildsettings_pass2.tag = m_tag_handle;
        buildsettings_pass2.pass = tri_mesh::BuildOffset::PASS_TWO;
        buildsettings_pass2.split_settings.split_boundary_edges = !m_lock_boundary;
        m_scheduler.add_operation_type<tri_mesh::BuildOffset>(
            "buildoffset_pass2",
            buildsettings_pass2);

        OperationSettings<tri_mesh::BuildOffsetPost> buildsettings_pass3;
        buildsettings_pass3.position = m_position_handle;
        buildsettings_pass3.tag = m_tag_handle;
        buildsettings_pass3.split_settings.split_boundary_edges = !m_lock_boundary;
        buildsettings_pass3.collapse_settings.collapse_boundary_edges = !m_lock_boundary;
        buildsettings_pass3.collapse_settings.collapse_boundary_vertex_to_interior = false;
        m_scheduler.add_operation_type<tri_mesh::BuildOffsetPost>(
            "buildoffset_pass3",
            buildsettings_pass3);
    }

    // remeshing and optimization
    {}
}

void IsosurfaceExtraction::process(const long iterations)
{
    // build offset
    m_scheduler.run_operation_on_all(PrimitiveType::Edge, "buildoffset_pass1");
    m_scheduler.run_operation_on_all(PrimitiveType::Edge, "buildoffset_pass2");
    m_scheduler.run_operation_on_all(PrimitiveType::Edge, "buildoffset_pass3");
    // for (long i = 0; i < iterations; ++i) {
    //     m_scheduler.run_operation_on_all(PrimitiveType::Edge, "split");
    //     m_scheduler.run_operation_on_all(PrimitiveType::Edge, "collapse");
    //     m_scheduler.run_operation_on_all(PrimitiveType::Edge, "swap");
    //     m_scheduler.run_operation_on_all(PrimitiveType::Vertex, "smooth");
    // }
}

} // namespace wmtk::components::internal
