#include "IsosurfaceExtraction.hpp"

#include <igl/edges.h>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/operations/tri_mesh/BuildOffset.hpp>
#include <wmtk/operations/tri_mesh/BuildOffsetPost.hpp>
#include <wmtk/operations/tri_mesh/EdgeCollapseToOptimal.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplitAtMidpoint.hpp>
#include <wmtk/operations/tri_mesh/EdgeSwap.hpp>
#include <wmtk/operations/tri_mesh/PushOffset.hpp>
#include <wmtk/operations/tri_mesh/VertexTangentialSmooth.hpp>

namespace wmtk::components::internal {

IsosurfaceExtraction::IsosurfaceExtraction(
    TriMesh& mesh,
    Eigen::MatrixXd& V,
    Eigen::MatrixXi& F,
    std::vector<bool>& Vtags,
    const double length,
    const bool lock_boundary)
    : m_mesh{mesh}
    , m_length_min{(4. / 5.) * length}
    , m_length_max{(4. / 3.) * length}
    , m_lock_boundary{lock_boundary}
    , m_scheduler(m_mesh)
{
    using namespace operations;
    // register the attributes
    m_position_handle = m_mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    m_tag_handle = m_mesh.get_attribute_handle<long>("tag", PrimitiveType::Vertex);

    auto tag = m_mesh.create_accessor(m_tag_handle);
    // set the tags
    size_t iter = 0;
    for (const Tuple& v : mesh.get_all(PrimitiveType::Vertex)) {
        if (Vtags[iter++])
            (tag.vector_attribute(v))(0) = INPUT;
        else
            (tag.vector_attribute(v))(0) = SCALFFOLD;
    }

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
    {
        // split
        // only for edges end without input vertices
        // OperationSettings<tri_mesh::EdgeSplitAtMidpoint> split_settings;
        // split_settings.position = m_position_handle;
        // split_settings.min_squared_length = m_length_max * m_length_max;
        // split_settings.split_settings.split_boundary_edges = !m_lock_boundary;
        // split_settings.initialize_invariants(m_mesh);
        // split_settings.for_extraction = true;
        // m_scheduler.add_operation_type<tri_mesh::EdgeSplitAtMidpoint>("split", split_settings);

        // collapse
        // only for exterior edges, edge should be collapse to offset
        // OperationSettings<tri_mesh::EdgeCollapseToMidpoint>;
        OperationSettings<tri_mesh::EdgeCollapseToOptimal> collapse_settings;
        collapse_settings.position = m_position_handle;
        collapse_settings.tag = m_tag_handle;
        collapse_settings.max_squared_length = m_length_min * m_length_min;
        collapse_settings.collapse_settings.collapse_boundary_edges = !m_lock_boundary;
        collapse_settings.collapse_towards_boundary = true;
        collapse_settings.initialize_invariants(m_mesh);
        m_scheduler.add_operation_type<tri_mesh::EdgeCollapseToOptimal>(
            "collapse",
            collapse_settings);

        // swap
        // for exterior edges, it would be normal
        // for interior edges, offset vertices should never be swapped
        // and interior offset should be offset if its edge's length could be smaller
        OperationSettings<tri_mesh::EdgeSwap> swap_settings;
        swap_settings.must_improve_valence = true;
        m_scheduler.add_operation_type<tri_mesh::EdgeSwap>("swap", swap_settings);

        // relocate
        // exterior vertices just do averaging
        // offset average neighbours' position and push back to offset vertices
        // OperationSettings<tri_mesh::VertexSmooth> relocate_pass1;
        // relocate_pass1.for_extraction = true;
        // relocate_pass1.position = m_position_handle;
        // relocate_pass1.tag = m_tag_handle;
        // relocate_pass1.smooth_boundary = !m_lock_boundary;
        // m_scheduler.add_operation_type<tri_mesh::VertexSmooth>("relocate_pass1", relocate_pass1);

        OperationSettings<tri_mesh::PushOffset> relocate_pass2;
        relocate_pass2.distance = length;
        relocate_pass2.position = m_position_handle;
        relocate_pass2.tag = m_tag_handle;
        relocate_pass2.smooth_boundary = !m_lock_boundary;
        m_scheduler.add_operation_type<tri_mesh::PushOffset>("relocate_pass2", relocate_pass2);
    }
}

void IsosurfaceExtraction::process(const long iteration_times)
{
    // build offset
    m_scheduler.run_operation_on_all(PrimitiveType::Edge, "buildoffset_pass1");
    m_scheduler.run_operation_on_all(PrimitiveType::Edge, "buildoffset_pass2");
    m_scheduler.run_operation_on_all(PrimitiveType::Edge, "buildoffset_pass3");
    for (long i = 0; i < iteration_times; ++i) {
        m_scheduler.run_operation_on_all(PrimitiveType::Edge, "split");
        m_scheduler.run_operation_on_all(PrimitiveType::Edge, "collapse");
        m_scheduler.run_operation_on_all(PrimitiveType::Edge, "swap");
        m_scheduler.run_operation_on_all(PrimitiveType::Edge, "relocate_pass1");
        m_scheduler.run_operation_on_all(PrimitiveType::Edge, "relocate_pass2");
    }
}

} // namespace wmtk::components::internal
