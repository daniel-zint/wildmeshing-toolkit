#include "MaxEdgeLengthInvariant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk {
MaxEdgeLengthInvariant::MaxEdgeLengthInvariant(
    const Mesh& m,
    const MeshAttributeHandle<double>& coordinate,
    double threshold_squared)
    : Invariant(m)
    , m_coordinate_handle(coordinate)
    , m_threshold_squared(threshold_squared)
{}
bool MaxEdgeLengthInvariant::before(const Simplex& t) const
{
    ConstAccessor<double> accessor = mesh().create_accessor(m_coordinate_handle);

    auto p0 = accessor.const_vector_attribute(t.tuple());
    auto p1 = accessor.const_vector_attribute(mesh().switch_vertex(t.tuple()));
    const double l_squared = (p1 - p0).squaredNorm();
    return l_squared < m_threshold_squared;
}
} // namespace wmtk
