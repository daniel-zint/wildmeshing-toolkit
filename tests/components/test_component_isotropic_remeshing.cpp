#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>
#include <wmtk/operations/OperationFactory.hpp>
#include <wmtk/operations/tri_mesh/EdgeCollapseToMidpoint.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplitAtMidpoint.hpp>
#include <wmtk/operations/tri_mesh/EdgeSwapValence.hpp>
#include <wmtk/operations/tri_mesh/VertexLaplacianSmooth.hpp>
#include <wmtk/operations/tri_mesh/VertexTangentialLaplacianSmooth.hpp>
#include <wmtk_components/input/input.hpp>
#include <wmtk_components/isotropic_remeshing/internal/IsotropicRemeshing.hpp>
#include <wmtk_components/isotropic_remeshing/internal/IsotropicRemeshingOptions.hpp>
#include <wmtk_components/isotropic_remeshing/isotropic_remeshing.hpp>
#include <wmtk_components/output/output.hpp>
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"


#include <catch2/catch_test_macros.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/multimesh/same_simplex_dimension_surjection.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include <wmtk/operations/tri_mesh/EdgeCollapse.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplit.hpp>
#include "../tools/DEBUG_EdgeMesh.hpp"
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/DEBUG_Tuple.hpp"
#include "../tools/EdgeMesh_examples.hpp"
#include "../tools/TriMesh_examples.hpp"

using json = nlohmann::json;
using namespace wmtk;
using namespace wmtk::tests;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

void print_tuple_map_iso(const DEBUG_TriMesh& parent, const DEBUG_MultiMeshManager& p_mul_manager)
{
    long child_id = 0;
    for (auto& child_data : p_mul_manager.children()) {
        std::cout << "child_id = " << child_id++ << std::endl;
        PrimitiveType map_ptype = child_data.mesh->top_simplex_type();
        auto parent_to_child_accessor = parent.create_accessor(child_data.map_handle);
        for (long parent_gid = 0; parent_gid < parent.capacity(map_ptype); ++parent_gid) {
            auto parent_to_child_data = parent_to_child_accessor.const_vector_attribute(
                parent.tuple_from_id(map_ptype, parent_gid));
            Tuple parent_tuple =
                wmtk::multimesh::utils::vector5_to_tuple(parent_to_child_data.head<5>());
            Tuple child_tuple =
                wmtk::multimesh::utils::vector5_to_tuple(parent_to_child_data.tail<5>());
            std::cout << "parent gid = " << parent_gid << std::endl;
            std::cout << "parent_tuple = " << wmtk::utils::TupleInspector::as_string(parent_tuple)
                      << std::endl;
            std::cout << "child_tuple = " << wmtk::utils::TupleInspector::as_string(child_tuple)
                      << std::endl
                      << std::endl;
        }
        std::cout << std::endl;
    }
}


TEST_CASE("smoothing_mesh", "[components][isotropic_remeshing][2D]")
{
    using namespace operations;

    std::map<std::string, std::filesystem::path> files;

    // input
    {
        json input_component_json = {
            {"type", "input"},
            {"name", "input_mesh"},
            {"cell_dimension", 2},
            {"file", (data_dir / "bumpyDice.msh").string()}};
        wmtk::components::input(input_component_json, files);
    }

    const std::filesystem::path& file = files["input_mesh"];
    auto mesh_in = wmtk::read_mesh(file);

    TriMesh& m = static_cast<TriMesh&>(*mesh_in);

    OperationSettings<tri_mesh::VertexLaplacianSmooth> op_settings(m);
    op_settings.position = mesh_in->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    Scheduler scheduler(m);
    scheduler.add_operation_type<tri_mesh::VertexLaplacianSmooth>("vertex_smooth", op_settings);

    for (int i = 0; i < 3; ++i) {
        scheduler.run_operation_on_all(PrimitiveType::Vertex, "vertex_smooth");
    }

    // output
    {
        ParaviewWriter writer("mesh_smooth", "vertices", m, true, true, true, false);
        m.serialize(writer);
    }
}

TEST_CASE("smoothing_simple_examples", "[components][isotropic_remeshing][2D]")
{
    using namespace operations;
    using namespace tri_mesh;

    SECTION("hex_plus_two")
    {
        DEBUG_TriMesh mesh = wmtk::tests::hex_plus_two_with_position();

        OperationSettings<VertexLaplacianSmooth> op_settings(mesh);
        op_settings.position = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

        // offset interior vertex
        auto pos = mesh.create_accessor(op_settings.position);
        Tuple v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);
        pos.vector_attribute(v4) = Eigen::Vector3d{0.6, 0.9, 0};

        Scheduler scheduler(mesh);
        scheduler.add_operation_type<VertexLaplacianSmooth>("vertex_smooth", op_settings);

        scheduler.run_operation_on_all(PrimitiveType::Vertex, "vertex_smooth");

        v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);
        Eigen::Vector3d after_smooth = pos.vector_attribute(v4);
        CHECK((after_smooth - Eigen::Vector3d{1, 0, 0}).squaredNorm() == 0);
    }

    SECTION("edge_region")
    {
        DEBUG_TriMesh mesh = wmtk::tests::edge_region_with_position();

        OperationSettings<VertexLaplacianSmooth> op_settings(mesh);
        op_settings.position = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

        // offset interior vertex
        auto pos = mesh.create_accessor(op_settings.position);
        Tuple v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);
        Tuple v5 = mesh.tuple_from_id(PrimitiveType::Vertex, 5);
        pos.vector_attribute(v4) = Eigen::Vector3d{0.6, 0.9, 0};
        pos.vector_attribute(v5) = Eigen::Vector3d{1.4, -0.9, 0};

        Scheduler scheduler(mesh);
        scheduler.add_operation_type<VertexLaplacianSmooth>("vertex_smooth", op_settings);

        for (size_t i = 0; i < 10; ++i) {
            scheduler.run_operation_on_all(PrimitiveType::Vertex, "vertex_smooth");
            v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);
            /*Eigen::Vector3d p4_after_smooth =*/pos.vector_attribute(v4);
        }

        Eigen::Vector3d p4_after_smooth = pos.vector_attribute(v4);
        CHECK((p4_after_smooth - Eigen::Vector3d{1, 0, 0}).squaredNorm() < 1e-10);

        v5 = mesh.tuple_from_id(PrimitiveType::Vertex, 5);
        Eigen::Vector3d p5_after_smooth = pos.vector_attribute(v5);
        CHECK((p5_after_smooth - Eigen::Vector3d{2, 0, 0}).squaredNorm() < 1e-10);
    }
}

TEST_CASE("tangential_smoothing", "[components][isotropic_remeshing][2D]")
{
    using namespace operations;
    using namespace tri_mesh;

    DEBUG_TriMesh mesh = wmtk::tests::hex_plus_two_with_position();

    OperationSettings<VertexTangentialLaplacianSmooth> op_settings(mesh);
    op_settings.position = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    // offset interior vertex
    auto pos = mesh.create_accessor(op_settings.position);
    Tuple v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);

    Eigen::Vector3d p_init;
    SECTION("1_0_1")
    {
        p_init = Eigen::Vector3d{1, 0, 1};
    }
    SECTION("0.5_0.5_1")
    {
        p_init = Eigen::Vector3d{0.5, 0.5, 1};
    }
    SECTION("0_0_7")
    {
        p_init = Eigen::Vector3d{0, 0, 7};
    }

    pos.vector_attribute(v4) = p_init;

    Scheduler scheduler(mesh);
    scheduler.add_operation_type<VertexTangentialLaplacianSmooth>(
        "vertex_tangential_smooth",
        op_settings);

    scheduler.run_operation_on_all(PrimitiveType::Vertex, "vertex_tangential_smooth");

    v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);
    Eigen::Vector3d after_smooth = pos.vector_attribute(v4);
    Eigen::Vector3d target = Eigen::Vector3d{1, 0, p_init[2]};
    std::cout << after_smooth.transpose() << " == " << target.transpose() << std::endl;
    CHECK((after_smooth - target).squaredNorm() == 0);
}

TEST_CASE("tangential_smoothing_boundary", "[components][isotropic_remeshing][2D]")
{
    using namespace operations;
    using namespace tri_mesh;

    DEBUG_TriMesh mesh = wmtk::tests::hex_plus_two_with_position();

    OperationSettings<VertexTangentialLaplacianSmooth> op_settings(mesh);
    op_settings.position = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    op_settings.smooth_boundary = true;

    op_settings.create_invariants();

    // offset interior vertex
    auto pos = mesh.create_accessor(op_settings.position);
    Tuple v1 = mesh.tuple_from_id(PrimitiveType::Vertex, 1);

    Eigen::Vector3d p_init;
    SECTION("1.7_1.1_0")
    {
        p_init = Eigen::Vector3d{1.7, 1.1, 0};
    }
    SECTION("2.2_2_0")
    {
        p_init = Eigen::Vector3d{2.2, 2, 0};
    }
    SECTION("2.2_2_5")
    {
        p_init = Eigen::Vector3d{2.2, 2, 5};
    }

    pos.vector_attribute(v1) = p_init;

    VertexTangentialLaplacianSmooth op(mesh, Simplex::vertex(v1), op_settings);
    const bool success = op();
    REQUIRE(success);

    v1 = mesh.tuple_from_id(PrimitiveType::Vertex, 1);
    Eigen::Vector3d after_smooth = pos.vector_attribute(v1);
    CHECK((after_smooth - Eigen::Vector3d{1.5, p_init[1], p_init[2]}).squaredNorm() == 0);
}

TEST_CASE("split_long_edges", "[components][isotropic_remeshing][split][2D]")
{
    using namespace operations;
    using namespace tri_mesh;

    // This test does not fully work yet

    DEBUG_TriMesh mesh = wmtk::tests::edge_region_with_position();

    OperationSettings<EdgeSplitAtMidpoint> op_settings(mesh);
    op_settings.position = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    {
        auto pos = mesh.create_accessor(op_settings.position);
        const Tuple v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);
        const Tuple v5 = mesh.tuple_from_id(PrimitiveType::Vertex, 5);
        // reposition interior vertices
        pos.vector_attribute(v4) = Eigen::Vector3d{0.6, 0.9, 0};
        pos.vector_attribute(v5) = Eigen::Vector3d{2.4, -0.9, 0};
        // std::cout << (Eigen::Vector3d{0.6, 0.9, 0} - Eigen::Vector3d{2.4, -0.9, 0}).squaredNorm()
        //           << std::endl;
    }

    SECTION("6.4")
    {
        //
        op_settings.min_squared_length = 6.4;

        Scheduler scheduler(mesh);
        scheduler.add_operation_type<tri_mesh::EdgeSplitAtMidpoint>(
            "tri_mesh_split_edge_at_midpoint",
            op_settings);

        size_t n_vertices = mesh.get_all(PrimitiveType::Vertex).size();
        size_t n_iterations = 0;
        for (; n_iterations < 10; ++n_iterations) {
            scheduler.run_operation_on_all(PrimitiveType::Edge, "tri_mesh_split_edge_at_midpoint");

            const size_t n_vertices_new = mesh.get_all(PrimitiveType::Vertex).size();
            if (n_vertices_new == n_vertices) {
                break;
            } else {
                n_vertices = n_vertices_new;
            }
        }

        CHECK(n_iterations == 1);
        REQUIRE(n_vertices == 11);

        // check position of new vertex
        auto pos = mesh.create_accessor(op_settings.position);
        const Tuple v10 = mesh.tuple_from_id(PrimitiveType::Vertex, 10);
        CHECK((pos.vector_attribute(v10) - Eigen::Vector3d{1.5, 0, 0}).squaredNorm() == 0);
    }
    SECTION("3.5")
    {
        //
        op_settings.min_squared_length = 3.5;

        Scheduler scheduler(mesh);
        scheduler.add_operation_type<tri_mesh::EdgeSplitAtMidpoint>(
            "tri_mesh_split_edge_at_midpoint",
            op_settings);

        size_t n_vertices = mesh.get_all(PrimitiveType::Vertex).size();
        size_t n_iterations = 0;
        for (; n_iterations < 10; ++n_iterations) {
            scheduler.run_operation_on_all(PrimitiveType::Edge, "tri_mesh_split_edge_at_midpoint");

            const size_t n_vertices_new = mesh.get_all(PrimitiveType::Vertex).size();
            if (n_vertices_new == n_vertices) {
                break;
            } else {
                n_vertices = n_vertices_new;
            }
        }

        CHECK(n_iterations < 5);
        CHECK(n_vertices == 15);
    }

    // check edge lengths
    auto pos = mesh.create_accessor(op_settings.position);
    for (const Tuple& e : mesh.get_all(PrimitiveType::Edge)) {
        const Eigen::Vector3d p0 = pos.vector_attribute(e);
        const Eigen::Vector3d p1 = pos.vector_attribute(mesh.switch_vertex(e));
        const double l_squared = (p1 - p0).squaredNorm();
        CHECK(l_squared < op_settings.min_squared_length);
    }
}

TEST_CASE("collapse_short_edges", "[components][isotropic_remeshing][collapse][2D]")
{
    using namespace operations;
    using namespace tri_mesh;

    DEBUG_TriMesh mesh = wmtk::tests::edge_region_with_position();

    OperationSettings<EdgeCollapseToMidpoint> op_settings(mesh);
    op_settings.position = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    SECTION("interior")
    {
        {
            auto pos = mesh.create_accessor(op_settings.position);
            const Tuple v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);
            const Tuple v5 = mesh.tuple_from_id(PrimitiveType::Vertex, 5);
            // reposition interior vertices
            pos.vector_attribute(v4) = Eigen::Vector3d{1.4, 0, 0};
            pos.vector_attribute(v5) = Eigen::Vector3d{1.6, 0, 0};
        }

        op_settings.max_squared_length = 0.1;

        Scheduler scheduler(mesh);
        scheduler.add_operation_type<EdgeCollapseToMidpoint>(
            "tri_mesh_collapse_edge_to_mid",
            op_settings);

        size_t n_vertices = mesh.get_all(PrimitiveType::Vertex).size();
        size_t n_iterations = 0;
        for (; n_iterations < 10; ++n_iterations) {
            scheduler.run_operation_on_all(PrimitiveType::Edge, "tri_mesh_collapse_edge_to_mid");

            const size_t n_vertices_new = mesh.get_all(PrimitiveType::Vertex).size();
            if (n_vertices_new == n_vertices) {
                break;
            } else {
                n_vertices = n_vertices_new;
            }
        }

        REQUIRE(n_iterations == 1);
        REQUIRE(n_vertices == 9);

        // CHECK_THROWS(mesh.tuple_from_id(PrimitiveType::Vertex, 4));
        const Tuple v5 = mesh.tuple_from_id(PrimitiveType::Vertex, 5);
        REQUIRE(mesh.is_valid_slow(v5));

        auto pos = mesh.create_accessor(op_settings.position);
        Eigen::Vector3d p5 = pos.vector_attribute(v5);
        CHECK((p5 - Eigen::Vector3d{1.5, 0, 0}).squaredNorm() == 0);
    }
    SECTION("towards_boundary_true")
    {
        {
            auto pos = mesh.create_accessor(op_settings.position);
            const Tuple v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);
            // reposition vertex
            pos.vector_attribute(v4) = Eigen::Vector3d{0.6, 0.9, 0};
        }

        op_settings.max_squared_length = 0.1;
        op_settings.collapse_towards_boundary = true;

        Scheduler scheduler(mesh);
        scheduler.add_operation_type<EdgeCollapseToMidpoint>(
            "tri_mesh_collapse_edge_to_mid",
            op_settings);

        size_t n_vertices = mesh.get_all(PrimitiveType::Vertex).size();
        size_t n_iterations = 0;
        for (; n_iterations < 10; ++n_iterations) {
            scheduler.run_operation_on_all(PrimitiveType::Edge, "tri_mesh_collapse_edge_to_mid");

            const size_t n_vertices_new = mesh.get_all(PrimitiveType::Vertex).size();
            if (n_vertices_new == n_vertices) {
                break;
            } else {
                n_vertices = n_vertices_new;
            }
        }

        REQUIRE(n_iterations == 1);
        REQUIRE(n_vertices == 9);

        CHECK_THROWS(mesh.tuple_from_id(PrimitiveType::Vertex, 4));
        const Tuple v0 = mesh.tuple_from_id(PrimitiveType::Vertex, 0);
        REQUIRE(mesh.is_valid_slow(v0));

        auto pos = mesh.create_accessor(op_settings.position);
        Eigen::Vector3d p0 = pos.vector_attribute(v0);
        CHECK((p0 - Eigen::Vector3d{0.5, 1, 0}).squaredNorm() == 0);
    }
    SECTION("towards_boundary_false")
    {
        {
            auto pos = mesh.create_accessor(op_settings.position);
            const Tuple v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);
            // reposition vertex
            pos.vector_attribute(v4) = Eigen::Vector3d{0.6, 0.9, 0};
        }

        op_settings.max_squared_length = 0.1;
        op_settings.collapse_towards_boundary = false;

        Scheduler scheduler(mesh);
        scheduler.add_operation_type<EdgeCollapseToMidpoint>(
            "tri_mesh_collapse_edge_to_mid",
            op_settings);

        size_t n_vertices = mesh.get_all(PrimitiveType::Vertex).size();
        size_t n_iterations = 0;
        for (; n_iterations < 10; ++n_iterations) {
            scheduler.run_operation_on_all(PrimitiveType::Edge, "tri_mesh_collapse_edge_to_mid");

            const size_t n_vertices_new = mesh.get_all(PrimitiveType::Vertex).size();
            if (n_vertices_new == n_vertices) {
                break;
            } else {
                n_vertices = n_vertices_new;
            }
        }

        REQUIRE(n_iterations == 1);
        REQUIRE(n_vertices == 9);

        CHECK_THROWS(mesh.tuple_from_id(PrimitiveType::Vertex, 4));
        const Tuple v0 = mesh.tuple_from_id(PrimitiveType::Vertex, 0);
        REQUIRE(mesh.is_valid_slow(v0));

        auto pos = mesh.create_accessor(op_settings.position);
        Eigen::Vector3d p0 = pos.vector_attribute(v0);
        CHECK((p0 - Eigen::Vector3d{0.55, 0.95, 0}).squaredNorm() == 0);
    }
    SECTION("collapse_boundary_true")
    {
        {
            auto pos = mesh.create_accessor(op_settings.position);
            const Tuple v1 = mesh.tuple_from_id(PrimitiveType::Vertex, 1);
            // reposition vertex
            pos.vector_attribute(v1) = Eigen::Vector3d{0.6, 1, 0};
        }

        op_settings.max_squared_length = 0.1;
        op_settings.collapse_boundary_edges = true;

        Scheduler scheduler(mesh);
        scheduler.add_operation_type<EdgeCollapseToMidpoint>(
            "tri_mesh_collapse_edge_to_mid",
            op_settings);

        scheduler.run_operation_on_all(PrimitiveType::Edge, "tri_mesh_collapse_edge_to_mid");

        REQUIRE(mesh.get_all(PrimitiveType::Vertex).size() == 9);

        CHECK_THROWS(mesh.tuple_from_id(PrimitiveType::Vertex, 1));
        const Tuple v0 = mesh.tuple_from_id(PrimitiveType::Vertex, 0);
        REQUIRE(mesh.is_valid_slow(v0));

        auto pos = mesh.create_accessor(op_settings.position);
        Eigen::Vector3d p0 = pos.vector_attribute(v0);
        CHECK((p0 - Eigen::Vector3d{0.55, 1, 0}).squaredNorm() == 0);
    }
    SECTION("collapse_boundary_false")
    {
        {
            auto pos = mesh.create_accessor(op_settings.position);
            const Tuple v1 = mesh.tuple_from_id(PrimitiveType::Vertex, 1);
            // reposition vertex
            pos.vector_attribute(v1) = Eigen::Vector3d{0.6, 1, 0};
        }

        op_settings.max_squared_length = 0.1;
        op_settings.collapse_boundary_edges = false;

        Scheduler scheduler(mesh);
        scheduler.add_operation_type<EdgeCollapseToMidpoint>(
            "tri_mesh_collapse_edge_to_mid",
            op_settings);

        scheduler.run_operation_on_all(PrimitiveType::Edge, "tri_mesh_collapse_edge_to_mid");

        REQUIRE(mesh.get_all(PrimitiveType::Vertex).size() == 10);
    }
}

TEST_CASE("swap_edge_for_valence", "[components][isotropic_remeshing][swap][2D]")
{
    using namespace operations;
    using namespace tri_mesh;

    DEBUG_TriMesh mesh = wmtk::tests::embedded_diamond();
    OperationSettings<EdgeSwapValence> op_settings(mesh);
    op_settings.create_invariants();

    Tuple swap_edge = mesh.edge_tuple_between_v1_v2(6, 7, 5);

    SECTION("single_op_fail")
    {
        EdgeSwapValence op(mesh, Simplex::edge(swap_edge), op_settings);
        CHECK_FALSE(op());
    }
    SECTION("swap_success")
    {
        // swap edge to create inbalence in valence
        {
            OperationSettings<tri_mesh::EdgeSwapBase> settings(mesh);
            settings.create_invariants();
            tri_mesh::EdgeSwapBase op(mesh, Simplex::edge(swap_edge), settings);
            REQUIRE(op());
            swap_edge = op.return_tuple();

            // check valence
            const Tuple v3 = mesh.tuple_from_id(PrimitiveType::Vertex, 3);
            const Tuple v6 = mesh.tuple_from_id(PrimitiveType::Vertex, 6);
            const Tuple v7 = mesh.tuple_from_id(PrimitiveType::Vertex, 7);
            const Tuple v10 = mesh.tuple_from_id(PrimitiveType::Vertex, 10);
            CHECK(SimplicialComplex::vertex_one_ring(mesh, v3).size() == 7);
            CHECK(SimplicialComplex::vertex_one_ring(mesh, v10).size() == 7);
            CHECK(SimplicialComplex::vertex_one_ring(mesh, v6).size() == 5);
            CHECK(SimplicialComplex::vertex_one_ring(mesh, v7).size() == 5);
        }


        SECTION("single_op")
        {
            EdgeSwapValence op(mesh, Simplex::edge(swap_edge), op_settings);
            CHECK(op.name() == "tri_mesh_edge_swap_valence");
            REQUIRE(op());
            swap_edge = op.return_tuple();
            CHECK(mesh.id(Simplex::vertex(swap_edge)) == 7);
            CHECK(mesh.id(Simplex::vertex(mesh.switch_vertex(swap_edge))) == 6);
        }
        SECTION("with_scheduler")
        {
            Scheduler scheduler(mesh);
            scheduler.add_operation_type<EdgeSwapValence>("TriMeshSwapEdgeOperation", op_settings);
            scheduler.run_operation_on_all(PrimitiveType::Edge, "TriMeshSwapEdgeOperation");
        }


        // check valence
        {
            const Tuple v3 = mesh.tuple_from_id(PrimitiveType::Vertex, 3);
            const Tuple v6 = mesh.tuple_from_id(PrimitiveType::Vertex, 6);
            const Tuple v7 = mesh.tuple_from_id(PrimitiveType::Vertex, 7);
            const Tuple v10 = mesh.tuple_from_id(PrimitiveType::Vertex, 10);
            CHECK(SimplicialComplex::vertex_one_ring(mesh, v3).size() == 6);
            CHECK(SimplicialComplex::vertex_one_ring(mesh, v10).size() == 6);
            CHECK(SimplicialComplex::vertex_one_ring(mesh, v6).size() == 6);
            CHECK(SimplicialComplex::vertex_one_ring(mesh, v7).size() == 6);
        }
    }
    SECTION("swap_fail")
    {
        OperationSettings<EdgeSwapValence> op_settings(mesh);
        op_settings.create_invariants();
        const Tuple e = mesh.edge_tuple_between_v1_v2(6, 7, 5);
        EdgeSwapValence op(mesh, Simplex::edge(e), op_settings);
        const bool success = op();
        CHECK(!success);
    }
}

TEST_CASE("component_isotropic_remeshing", "[components][isotropic_remeshing][2D][.]")
{
    std::map<std::string, std::filesystem::path> files;
    {
        json input_component_json = {
            {"type", "input"},
            {"name", "input_mesh"},
            {"cell_dimension", 2},
            {"file", data_dir / "bunny.off"}};
        REQUIRE_NOTHROW(wmtk::components::input(input_component_json, files));
    }

    json mesh_isotropic_remeshing_json = {
        {"type", "isotropic_remeshing"},
        {"input", "input_mesh"},
        {"output", "output_mesh"},
        {"length_abs", 0.003},
        {"length_rel", -1},
        {"iterations", 1},
        {"lock_boundary", true}};
    REQUIRE_NOTHROW(wmtk::components::isotropic_remeshing(mesh_isotropic_remeshing_json, files));

    //{
    //    json component_json = {
    //        {"type", "output"},
    //        {"input", "output_mesh"},
    //        {"file", "bunny_isotropic_remeshing"}};
    //
    //    CHECK_NOTHROW(wmtk::components::output(component_json, files));
    //}
}

TEST_CASE("remeshing_tetrahedron", "[components][isotropic_remeshing][2D][.]")
{
    using namespace wmtk::components::internal;

    // input
    TriMesh mesh = tetrahedron_with_position();

    IsotropicRemeshing
        isotropicRemeshing(mesh, 0.5, true, false, false, true, true, true, true, false);
    isotropicRemeshing.remeshing(20);

    ParaviewWriter writer("tet_remeshing", "vertices", mesh, true, true, true, false);
    mesh.serialize(writer);
}

TEST_CASE("remeshing_with_boundary", "[components][isotropic_remeshing][2D][.]")
{
    using namespace wmtk::components::internal;

    // input
    TriMesh mesh = edge_region_with_position();

    SECTION("lock_boundary_false")
    {
        IsotropicRemeshing
            isotropicRemeshing(mesh, 0.5, false, false, false, true, true, true, true, false);
        isotropicRemeshing.remeshing(5);

        size_t n_boundary_edges = 0;
        for (const Tuple& e : mesh.get_all(PrimitiveType::Edge)) {
            if (mesh.is_boundary_edge(e)) {
                ++n_boundary_edges;
            }
        }
        CHECK(n_boundary_edges > 8);
    }

    SECTION("lock_boundary_true")
    {
        IsotropicRemeshing
            isotropicRemeshing(mesh, 0.5, true, false, false, true, true, true, true, false);
        isotropicRemeshing.remeshing(5);

        size_t n_boundary_edges = 0;
        for (const Tuple& e : mesh.get_all(PrimitiveType::Edge)) {
            if (mesh.is_boundary_edge(e)) {
                ++n_boundary_edges;
            }
        }
        CHECK(n_boundary_edges == 8);

        // ParaviewWriter writer("w_bd_remeshing", "vertices", mesh, true, true, true, false);
        // mesh.serialize(writer);
    }
}

TEST_CASE("remeshing_preserve_topology", "[components][isotropic_remeshing][2D]")
{
    using namespace wmtk::components::internal;

    // input
    DEBUG_TriMesh mesh = edge_region_with_position();
    // DEBUG_TriMesh mesh = hex_plus_two_with_position();
    auto tag_handle = mesh.register_attribute<long>("is_boundary", wmtk::PrimitiveType::Edge, 1);
    auto tag_accessor = mesh.create_accessor(tag_handle);
    for (const Tuple& e : mesh.get_all(PrimitiveType::Edge)) {
        if (mesh.is_boundary_edge(e)) {
            tag_accessor.scalar_attribute(e) = 1;
        } else {
            tag_accessor.scalar_attribute(e) = 0;
        }
    }
    std::shared_ptr<Mesh> child_ptr =
        wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            mesh,
            "is_boundary",
            1,
            PrimitiveType::Edge);

    REQUIRE(mesh.get_child_meshes().size() == 1);
    mesh.multi_mesh_manager().check_map_valid(mesh);
    const auto& child_mesh = *child_ptr;
    CHECK(child_mesh.get_all(PrimitiveType::Edge).size() == 8);
    CHECK(child_mesh.get_all(PrimitiveType::Vertex).size() == 8);


    IsotropicRemeshing
        isotropicRemeshing(mesh, 0.5, /*lock_boundary*/false, /*preserve_childmesh_Topology*/true, /*preserve_Childmesh_geometry*/false, /*do_Split*/true, /*do_collapse*/true, /*do_swap*/true, /*do_smooth*/true, /*debug_output*/false);
    isotropicRemeshing.remeshing(5);
    REQUIRE(mesh.is_connectivity_valid());
    mesh.multi_mesh_manager().check_map_valid(mesh);


    size_t n_boundary_edges = 0;
    for (const Tuple& e : mesh.get_all(PrimitiveType::Edge)) {
        if (mesh.is_boundary_edge(e)) {
            ++n_boundary_edges;
        }
    }
    // CHECK(n_boundary_edges > 8);

    // output
    {
        ParaviewWriter writer("remeshing_test", "vertices", mesh, true, true, true, false);
        mesh.serialize(writer);
    }
}

TEST_CASE("remeshing_preserve_topology_realmesh", "[components][isotropic_remeshing][2D][.]")
{
    using namespace wmtk::components::internal;
    using namespace operations;

    std::map<std::string, std::filesystem::path> files;

    // input
    // TODO: What is the default attribute for "vertices". From the reader it seems to be
    // "vertices". need change "vertices" to "vertices" isotropic_remeshing.hpp
    {
        json input_component_json = {
            {"type", "input"},
            {"name", "input_mesh"},
            {"cell_dimension", 2},
            {"file", (data_dir / "circle.msh").string()}};
        wmtk::components::input(input_component_json, files);
    }

    const std::filesystem::path& file = files["input_mesh"];
    auto m = wmtk::read_mesh(file);
    tests::DEBUG_TriMesh& mesh = static_cast<tests::DEBUG_TriMesh&>(*m);

    auto tag_handle = mesh.register_attribute<long>("is_boundary", wmtk::PrimitiveType::Edge, 1);
    auto tag_accessor = mesh.create_accessor(tag_handle);
    for (const Tuple& e : mesh.get_all(PrimitiveType::Edge)) {
        if (mesh.is_boundary_edge(e)) {
            tag_accessor.scalar_attribute(e) = 1;
        } else {
            tag_accessor.scalar_attribute(e) = 0;
        }
    }
    std::shared_ptr<Mesh> child_ptr =
        wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            mesh,
            "is_boundary",
            1,
            PrimitiveType::Edge);

    REQUIRE(mesh.get_child_meshes().size() == 1);
    // mesh.multi_mesh_manager().check_map_valid(mesh);
    // const auto& child_mesh = *child_ptr;

    IsotropicRemeshing
        isotropicRemeshing(mesh, 0.05, false, false, false, true, true, true, true, false);
    // IsotropicRemeshing isotropicRemeshing(mesh, 0.5, false, false, false);

    for (int i = 0; i < 25; i++) {
        isotropicRemeshing.remeshing(1);
        std::cout << "finish remeshing iter " << i << std::endl;
        REQUIRE(mesh.is_connectivity_valid());
        mesh.multi_mesh_manager().check_map_valid(mesh);
        std::cout << "finish checking" << std::endl;
    }


    auto child_vertex_handle =
        child_ptr->register_attribute<double>("vertices", wmtk::PrimitiveType::Vertex, 3);
    auto child_vertex_accessor = child_ptr->create_accessor(child_vertex_handle);

    auto parent_vertex_handle =
        mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto parent_vertex_accessor = mesh.create_accessor(parent_vertex_handle);

    std::cout << "finish create handle" << std::endl;


    for (const auto v : child_ptr->get_all(PrimitiveType::Vertex)) {
        auto parent_v = child_ptr->map_to_root_tuple(Simplex(PrimitiveType::Vertex, v));
        child_vertex_accessor.vector_attribute(v) =
            parent_vertex_accessor.vector_attribute(parent_v);
        // std::cout << parent_vertex_accessor.vector_attribute(parent_v) << std::endl;
    }
    std::cout << "finish position write" << std::endl;


    // output
    {
        ParaviewWriter
            writer("remeshing_test_circle_final", "vertices", mesh, true, true, true, false);
        mesh.serialize(writer);

        ParaviewWriter writer2(
            "remeshing_test_circle_child_mesh_final",
            "vertices",
            *child_ptr,
            true,
            true,
            false,
            false);
        child_ptr->serialize(writer2);
    }
}

TEST_CASE("remeshing_realmesh", "[components][isotropic_remeshing][2D][.]")
{
    using namespace wmtk::components::internal;
    using namespace operations;

    std::map<std::string, std::filesystem::path> files;

    // input

    {
        json input_component_json = {
            {"type", "input"},
            {"name", "input_mesh"},
            {"cell_dimension", 2},
            {"file", (data_dir / "circle.msh").string()}};
        wmtk::components::input(input_component_json, files);
    }

    const std::filesystem::path& file = files["input_mesh"];
    auto m = wmtk::read_mesh(file);
    TriMesh& mesh = static_cast<TriMesh&>(*m);

    // auto tag_handle = mesh.register_attribute<long>("is_boundary", wmtk::PrimitiveType::Edge, 1);
    // auto tag_accessor = mesh.create_accessor(tag_handle);
    // for (const Tuple& e : mesh.get_all(PrimitiveType::Edge)) {
    //     if (mesh.is_boundary_edge(e)) {
    //         tag_accessor.scalar_attribute(e) = 1;
    //     } else {
    //         tag_accessor.scalar_attribute(e) = 0;
    //     }
    // }
    // std::shared_ptr<Mesh> child_ptr =
    //     wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
    //         mesh,
    //         "is_boundary",
    //         1,
    //         PrimitiveType::Edge);

    // REQUIRE(mesh.get_child_meshes().size() == 1);
    // mesh.multi_mesh_manager().check_map_valid(mesh);
    // const auto& child_mesh = *child_ptr;

    IsotropicRemeshing
        isotropicRemeshing(mesh, 0.5, false, false, false, true, true, true, true, false);
    isotropicRemeshing.remeshing(25);
    std::cout << "finish remeshing" << std::endl;
    REQUIRE(mesh.is_connectivity_valid());
    // mesh.multi_mesh_manager().check_map_valid(mesh);
    std::cout << "finish checking" << std::endl;


    // output
    {
        ParaviewWriter
            writer("remeshing_test_circle_no_nultimesh", "vertices", mesh, true, true, true, false);
        mesh.serialize(writer);
    }
}
