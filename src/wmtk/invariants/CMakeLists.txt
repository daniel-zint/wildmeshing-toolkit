set(SRC_FILES
    Invariant.hpp
    Invariant.cpp
    TriMeshInvariant.hpp
    TriMeshInvariant.cpp
    TetMeshInvariant.hpp
    TetMeshInvariant.cpp
    MeshInvariant.hpp
    MeshInvariant.cpp
    InvariantCollection.hpp
    InvariantCollection.cpp
    InteriorSimplexInvariant.hpp
    InteriorSimplexInvariant.cpp
    InteriorVertexInvariant.hpp
    InteriorVertexInvariant.cpp
    InteriorEdgeInvariant.hpp
    InteriorEdgeInvariant.cpp
    MultiMeshLinkConditionInvariant.hpp
    MultiMeshLinkConditionInvariant.cpp
    MultiMeshMapValidInvariant.hpp
    MultiMeshMapValidInvariant.cpp
    MaxEdgeLengthInvariant.hpp
    MaxEdgeLengthInvariant.cpp
    MinEdgeLengthInvariant.hpp
    MinEdgeLengthInvariant.cpp

    MultiMeshTopologyInvariant.hpp
    MultiMeshTopologyInvariant.cpp
    MinIncidentValenceInvariant.hpp
    MinIncidentValenceInvariant.cpp
    TriMeshSubstructureTopologyPreservingInvariant.hpp
    TriMeshSubstructureTopologyPreservingInvariant.cpp
    TetMeshSubstructureTopologyPreservingInvariant.hpp
    TetMeshSubstructureTopologyPreservingInvariant.cpp
    TriangleInversionInvariant.hpp
    TriangleInversionInvariant.cpp
    TodoInvariant.hpp
    TodoInvariant.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
