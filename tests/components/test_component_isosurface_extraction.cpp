#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"

using json = nlohmann::json;
using namespace wmtk;
using namespace wmtk::tests;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("isosurface_extraction_dummy", "[.]")
{
    // TODO
    REQUIRE(false);
}