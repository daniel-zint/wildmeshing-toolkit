# Include Catch2 stuff

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/recipes/tests")

include(catch2)
include(wmtk_data)


# Sources
set(TEST_SOURCES
    test_topology.cpp
    test_mesh.cpp
    test_autogen.cpp
    test_boundary.cpp
    test_tuple.cpp
    test_tuple_1d.cpp
    test_tuple_2d.cpp
	test_tuple_3d.cpp
    test_execution.cpp
    test_invariants.cpp
    test_simplex_collection.cpp
    test_simplicial_complex.cpp
    test_1d_operations.cpp
    test_2d_operations.cpp
    test_2d_operation_construction.cpp
    test_accessor.cpp
    test_example_meshes.cpp
    test_3d_operations.cpp
    test_mesh_variant.cpp
    test_variant_metaprogramming.cpp
    test_autodiff.cpp
    test_tuple_metaprogramming.cpp
    test_local_switch_search.cpp
    test_primitive.cpp

    tools/all_valid_local_tuples.hpp
    tools/all_valid_local_tuples.cpp
    tools/DEBUG_PointMesh.hpp
    tools/DEBUG_EdgeMesh.hpp
    tools/DEBUG_EdgeMesh.cpp
    tools/DEBUG_TriMesh.hpp
    tools/DEBUG_TriMesh.cpp
    tools/EdgeMesh_examples.hpp
    tools/EdgeMesh_examples.cpp
    tools/TriMesh_examples.hpp
    tools/TriMesh_examples.cpp
    tools/redirect_logger_to_cout.hpp
    tools/TetMesh_examples.hpp
    tools/TetMesh_examples.cpp
    tools/DEBUG_TetMesh.hpp
    tools/DEBUG_TetMesh.cpp

    tools/DEBUG_Mesh.hpp
    tools/DEBUG_Mesh.cpp
    tools/DEBUG_MultiMeshManager.hpp
    tools/DEBUG_MultiMeshManager.cpp
    tools/DEBUG_Tuple.hpp
    tools/DEBUG_Tuple.cpp
)
add_executable(wmtk_tests ${TEST_SOURCES})


# Dependencies
include(finite-diff)

target_link_libraries(wmtk_tests PUBLIC
    wmtk::toolkit
    wmtk::data
    Catch2::Catch2WithMain
    finitediff::finitediff
)

FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)

function(add_subdirectory_with_source_group)
    set(SUBDIR_NAME "${ARGV0}")
    add_subdirectory(${SUBDIR_NAME})
    source_group("${SUBDIR_NAME}" REGULAR_EXPRESSION "${SUBDIR_NAME}\/.*\.(cpp|h|hpp)?$")
endfunction()

add_subdirectory_with_source_group(components)
add_subdirectory_with_source_group(function)
add_subdirectory_with_source_group(simplex)
add_subdirectory_with_source_group(io)



add_subdirectory(multimesh)
source_group("multimesh" REGULAR_EXPRESSION "multimesh\/.*\.(cpp|h|hpp)?$")

wmtk_copy_dll(wmtk_tests)

# Register unit tests
catch_discover_tests(wmtk_tests)
