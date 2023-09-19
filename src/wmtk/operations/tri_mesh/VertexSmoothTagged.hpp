#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TriMeshOperation.hpp"
#include "VertexSmooth.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class VertexSmoothTagged;
}

template <>
struct OperationSettings<tri_mesh::VertexSmoothTagged>
{
    OperationSettings<tri_mesh::VertexSmooth> smooth_settings;
};

namespace tri_mesh {
class VertexSmoothTagged : public TriMeshOperation, private TupleOperation
{
public:
    VertexSmoothTagged(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<VertexSmoothTagged>& settings);

    std::string name() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Vertex; }

protected:
    bool before() const override;
    bool execute() override;

private:
    Accessor<double> m_pos_accessor;
    Accessor<long> m_tag_accessor;
    const OperationSettings<VertexSmoothTagged>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
