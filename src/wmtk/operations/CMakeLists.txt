set(SRC_FILES
    Operation.hpp
    Operation.cpp
    TupleOperation.hpp
    TupleOperation.cpp
    OperationFactory.hpp
    OperationFactory.cpp
    OperationQueue.hpp
    OperationSettings.hpp
    OperationSettingsBase.cpp
    OperationSettingsBase.hpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})

add_subdirectory(edge_mesh)
add_subdirectory(tri_mesh)
add_subdirectory(tet_mesh)
add_subdirectory(utils)

