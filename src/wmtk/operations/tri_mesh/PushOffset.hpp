#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TriMeshOperation.hpp"
#include "VertexSmooth.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class PushOffset;
}

template <>
struct OperationSettings<tri_mesh::PushOffset>
{
    MeshAttributeHandle<double> position;
    MeshAttributeHandle<long> tag;
    bool smooth_boundary = false;
    InvariantCollection invariants;
    double distance = 0.5;
};

namespace tri_mesh {
class PushOffset : public TriMeshOperation, private TupleOperation
{
public:
    PushOffset(Mesh& m, const Tuple& t, const OperationSettings<PushOffset>& settings);

    std::string name() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Vertex; }

    const Tuple& return_tuple() const;

    Eigen::Vector3d getClosestPointFromEdge(const Tuple& t, const Tuple& et0, const Tuple& et1);

    Eigen::Vector3d computeOriginOnInput(const Tuple& t, const std::vector<Tuple> neighbours);

protected:
    bool before() const override;
    bool execute() override;

private:
    Tuple m_output_tuple;
    Accessor<double> m_pos_accessor;
    Accessor<long> m_tag_accessor;
    const OperationSettings<PushOffset>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
