#pragma once
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "EdgeCollapse.hpp"
#include "EdgeSplit.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class BuildOffsetPost;
} // namespace tri_mesh

template <>
struct OperationSettings<tri_mesh::BuildOffsetPost>
{
    OperationSettings<tri_mesh::EdgeSplit> split_settings;
    OperationSettings<tri_mesh::EdgeCollapse> collapse_settings;
    // handle to vertex position
    MeshAttributeHandle<double> position;
    // handle to vertex tag
    MeshAttributeHandle<int> tag;
    // debug functionality to make sure operations are constructed properly
    // bool are_invariants_initialized() const;
};

namespace tri_mesh {
class BuildOffsetPost : public TriMeshOperation, private TupleOperation
{
public:
    BuildOffsetPost(Mesh& m, const Tuple& t, const OperationSettings<BuildOffsetPost>& settings);

    std::string name() const override;

    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Face; }

protected:
    bool before() const override;
    bool execute() override;

private:
    Tuple m_output_tuple;
    Accessor<double> m_pos_accessor;
    Accessor<int> m_tag_accessor; // 0-scalffold 1-input 2-offset

    const OperationSettings<BuildOffsetPost>& m_settings;

    Eigen::Vector3d p0;
    Eigen::Vector3d p1;
    Eigen::Vector3d p2;
    int t0;
    int t1;
    int t2;
};

} // namespace tri_mesh
} // namespace wmtk::operations
