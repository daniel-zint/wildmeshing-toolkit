#pragma once
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "EdgeSplit.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class BuildOffset;
} // namespace tri_mesh

template <>
struct OperationSettings<tri_mesh::BuildOffset>
{
    OperationSettings<tri_mesh::EdgeSplit> split_settings;
    // handle to vertex position
    MeshAttributeHandle<double> position;
    // handle to vertex tag
    MeshAttributeHandle<long> tag;

    int pass = -1;

    // debug functionality to make sure operations are constructed properly
    // bool are_invariants_initialized() const;
};

namespace tri_mesh {
class BuildOffset : public TriMeshOperation, private TupleOperation
{
public:
    BuildOffset(Mesh& m, const Tuple& t, const OperationSettings<BuildOffset>& settings);

    std::string name() const override;

    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

    enum PassNum { PASS_ONE, PASS_TWO };

protected:
    bool before() const override;
    bool execute() override;

private:
    Tuple m_output_tuple;
    Accessor<double> m_pos_accessor;
    Accessor<long> m_tag_accessor; // 0-scalffold 1-input 2-offset

    const OperationSettings<BuildOffset>& m_settings;

    Eigen::Vector3d p0;
    Eigen::Vector3d p1;
    long t0;
    long t1;
};

} // namespace tri_mesh
} // namespace wmtk::operations
