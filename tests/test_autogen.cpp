#include <spdlog/spdlog.h>
#include <stdlib.h>
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <tuple>
#include <wmtk/Tuple.hpp>
#include <wmtk/autogen/edge_mesh/is_ccw.hpp>
#include <wmtk/autogen/edge_mesh/local_switch_tuple.hpp>
#include <wmtk/autogen/is_ccw.hpp>
#include <wmtk/autogen/local_switch_tuple.hpp>
#include <wmtk/autogen/tet_mesh/autogenerated_tables.hpp>
#include <wmtk/autogen/tet_mesh/is_ccw.hpp>
#include <wmtk/autogen/tet_mesh/local_id_table_offset.hpp>
#include <wmtk/autogen/tet_mesh/local_switch_tuple.hpp>
#include <wmtk/autogen/tri_mesh/autogenerated_tables.hpp>
#include <wmtk/autogen/tri_mesh/is_ccw.hpp>
#include <wmtk/autogen/tri_mesh/local_id_table_offset.hpp>
#include <wmtk/autogen/tri_mesh/local_switch_tuple.hpp>

using namespace wmtk;
using namespace wmtk::autogen;
;
namespace {

std::vector<PrimitiveType> primitives_up_to(PrimitiveType pt)
{
    std::vector<PrimitiveType> r;

    switch (pt) {
    case PrimitiveType::Tetrahedron: r.emplace_back(PrimitiveType::Face); [[fallthrough]];
    case PrimitiveType::Face: r.emplace_back(PrimitiveType::Edge); [[fallthrough]];
    case PrimitiveType::Edge: r.emplace_back(PrimitiveType::Vertex); [[fallthrough]];
    case PrimitiveType::Vertex:
    default: break;
    }
    return r;
}


long max_tuple_count(PrimitiveType pt)
{
    switch (pt) {
    case PrimitiveType::Face: return long(std::size(wmtk::autogen::tri_mesh::auto_2d_table_ccw));
    case PrimitiveType::Tetrahedron:
        return long(std::size(wmtk::autogen::tet_mesh::auto_3d_table_ccw));
    case PrimitiveType::Edge: return 2;
    case PrimitiveType::Vertex: break;
    }
    return -1;
}

Tuple tuple_from_offset_id(PrimitiveType pt, int offset)
{
    long lvid = 0, leid = 0, lfid = 0, gcid = 0, hash = 0;

    switch (pt) {
    case PrimitiveType::Face: {
        // bug in the standard? tie should work :-(
        auto r = tri_mesh::lvid_leid_from_table_offset(offset);
        lvid = r[0];
        leid = r[1];
    } break;
    case PrimitiveType::Tetrahedron: {
        auto r = tet_mesh::lvid_leid_lfid_from_table_offset(offset);
        lvid = r[0];
        leid = r[1];
        lfid = r[2];
    } break;
    case PrimitiveType::Edge: {
        lvid = offset;
    } break;
    case PrimitiveType::Vertex: break;
    }

    Tuple r(lvid, leid, lfid, gcid, hash);
    if (!tuple_is_valid_for_ccw(pt, r)) {
        r = Tuple();
    }
    return r;
}

std::vector<Tuple> all_valid_local_tuples(PrimitiveType pt)
{
    std::vector<Tuple> tups;
    tups.reserve(max_tuple_count(pt));
    for (long idx = 0; idx < max_tuple_count(pt); ++idx) {
        tups.emplace_back(tuple_from_offset_id(pt, idx));
    }

    tups.erase(
        std::remove_if(
            tups.begin(),
            tups.end(),
            [](const Tuple& t) -> bool { return t.is_null(); }),
        tups.end());
    return tups;
}
} // namespace

TEST_CASE("tuple_autogen_sizes", "[tuple]")
{
    size_t valid_face = 6;
    size_t valid_tet = 24;

    REQUIRE(all_valid_local_tuples(PrimitiveType::Face).size() == valid_face);
    REQUIRE(all_valid_local_tuples(PrimitiveType::Tetrahedron).size() == valid_tet);

    auto get_array_range = [](const auto& array) -> std::array<decltype(&array[0]), 2> {
        return std::array<decltype(&array[0]), 2>{{array, array + std::size(array)}};
    };
    {// ccw check
     {// tri
      auto ccw_range = get_array_range(tri_mesh::auto_2d_table_ccw);
    size_t count = std::count_if(ccw_range[0], ccw_range[1], [](long v) { return v != -1; });
    CHECK(count == valid_face);
}
{
    auto ccw_range = get_array_range(tet_mesh::auto_3d_table_ccw);
    size_t count = std::count_if(ccw_range[0], ccw_range[1], [](long v) { return v != -1; });
    CHECK(count == valid_tet);
}
}
{{// tri
  auto range = get_array_range(tri_mesh::auto_2d_table_vertex);
size_t count =
    std::count_if(range[0], range[1], [](const long v[2]) { return v[0] != -1 && v[1] != -1; });
CHECK(count == valid_face);
}
{ // tri
    auto range = get_array_range(tri_mesh::auto_2d_table_edge);
    size_t count =
        std::count_if(range[0], range[1], [](const long v[2]) { return v[0] != -1 && v[1] != -1; });
    CHECK(count == valid_face);
}
}
{
    { // tet
        auto range = get_array_range(tet_mesh::auto_3d_table_vertex);
        size_t count = std::count_if(range[0], range[1], [](const long v[3]) {
            return v[0] != -1 && v[1] != -1 && v[2] != -1;
        });
        CHECK(count == valid_tet);
    }
    { // tet
        auto range = get_array_range(tet_mesh::auto_3d_table_edge);
        size_t count = std::count_if(range[0], range[1], [](const long v[3]) {
            return v[0] != -1 && v[1] != -1 && v[2] != -1;
        });
        CHECK(count == valid_tet);
    }
    { // tet
        auto range = get_array_range(tet_mesh::auto_3d_table_face);
        size_t count = std::count_if(range[0], range[1], [](const long v[3]) {
            return v[0] != -1 && v[1] != -1 && v[2] != -1;
        });
        CHECK(count == valid_tet);
    }
}
}

TEST_CASE("tuple_autogen_id_inversion", "[tuple]")
{
    // when other meshes are available add them here
    for (PrimitiveType pt : {PrimitiveType::Face, PrimitiveType::Tetrahedron}) {
        for (long idx = 0; idx < max_tuple_count(pt); ++idx) {
            Tuple t = tuple_from_offset_id(pt, idx);
            if (t.is_null()) {
                continue;
            } else {
                switch (pt) {
                case PrimitiveType::Face: {
                    CHECK(idx == tri_mesh::local_id_table_offset(t));
                    break;
                }
                case PrimitiveType::Tetrahedron: {
                    CHECK(idx == tet_mesh::local_id_table_offset(t));
                    break;
                }
                case PrimitiveType::Vertex:
                case PrimitiveType::Edge: break;
                }
            }
        }
    }
}

TEST_CASE("tuple_autogen_ptype_is_ccw_equivalent", "[tuple]")
{
    {
        auto tuples = all_valid_local_tuples(PrimitiveType::Edge);
        for (const auto& t : tuples) {
            CHECK(edge_mesh::is_ccw(t) == is_ccw(PrimitiveType::Edge, t));
        }
    }

    {
        auto tuples = all_valid_local_tuples(PrimitiveType::Face);
        for (const auto& t : tuples) {
            CHECK(tri_mesh::is_ccw(t) == is_ccw(PrimitiveType::Face, t));
        }
    }

    {
        auto tuples = all_valid_local_tuples(PrimitiveType::Tetrahedron);
        for (const auto& t : tuples) {
            CHECK(tet_mesh::is_ccw(t) == is_ccw(PrimitiveType::Tetrahedron, t));
        }
    }
}

TEST_CASE("tuple_autogen_local_id_inversion", "[tuple]")
{
    // NOTE: this works because we assume the unused ids are = 0; from tuple_from_offset_id
    // above
    {
        auto tuples = all_valid_local_tuples(PrimitiveType::Face);
        for (const auto& t : tuples) {
            long id = tri_mesh::local_id_table_offset(t);
            auto [lvid, leid] = tri_mesh::lvid_leid_from_table_offset(id);
            Tuple nt(lvid, leid, 0, 0, 0);
            long nid = tri_mesh::local_id_table_offset(nt);

            CHECK(t == nt);
            CHECK(id == nid);
        }
    }

    {
        auto tuples = all_valid_local_tuples(PrimitiveType::Tetrahedron);
        for (const auto& t : tuples) {
            long id = tet_mesh::local_id_table_offset(t);
            auto [lvid, leid, lfid] = tet_mesh::lvid_leid_lfid_from_table_offset(id);
            Tuple nt(lvid, leid, lfid, 0, 0);
            long nid = tet_mesh::local_id_table_offset(nt);
            CHECK(t == nt);
            CHECK(id == nid);
        }
    }
}

TEST_CASE("tuple_autogen_ptype_local_switch_tuple_equivalent", "[tuple]")
{
    {
        auto tuples = all_valid_local_tuples(PrimitiveType::Edge);
        for (const auto& t : tuples) {
            for (PrimitiveType pt : primitives_up_to(PrimitiveType::Edge)) {
                CHECK(
                    edge_mesh::local_switch_tuple(t, pt) ==
                    local_switch_tuple(PrimitiveType::Edge, t, pt));
            }
            REQUIRE_THROWS(edge_mesh::local_switch_tuple(t, PrimitiveType::Tetrahedron));
            REQUIRE_THROWS(edge_mesh::local_switch_tuple(t, PrimitiveType::Face));
            REQUIRE_THROWS(edge_mesh::local_switch_tuple(t, PrimitiveType::Edge));
        }
    }
    {
        auto tuples = all_valid_local_tuples(PrimitiveType::Face);
        for (const auto& t : tuples) {
            for (PrimitiveType pt : primitives_up_to(PrimitiveType::Face)) {
                CHECK(
                    tri_mesh::local_switch_tuple(t, pt) ==
                    local_switch_tuple(PrimitiveType::Face, t, pt));
            }
            REQUIRE_THROWS(tri_mesh::local_switch_tuple(t, PrimitiveType::Tetrahedron));
            REQUIRE_THROWS(tri_mesh::local_switch_tuple(t, PrimitiveType::Face));
        }
    }

    {
        auto tuples = all_valid_local_tuples(PrimitiveType::Tetrahedron);
        for (const auto& t : tuples) {
            for (PrimitiveType pt : primitives_up_to(PrimitiveType::Tetrahedron)) {
                CHECK(
                    tet_mesh::local_switch_tuple(t, pt) ==
                    local_switch_tuple(PrimitiveType::Tetrahedron, t, pt));
            }
            REQUIRE_THROWS(tet_mesh::local_switch_tuple(t, PrimitiveType::Tetrahedron));
        }
    }
}


TEST_CASE("tuple_autogen_switch_still_valid", "[tuple]")
{
    // when other meshes are available add them here
    for (PrimitiveType mesh_type : {PrimitiveType::Face /*, PrimitiveType::Tetrahedron*/}) {
        auto tuples = all_valid_local_tuples(mesh_type);

        for (const auto& t : tuples) {
            CHECK(tuple_is_valid_for_ccw(mesh_type, t));
            // for (PrimitiveType pt : primitives_up_to(mesh_type)) {
            //     CHECK(tuple_is_valid_for_ccw(mesh_type, local_switch_tuple(mesh_type, t,
            //     pt)));
            // }
        }
    }
}
