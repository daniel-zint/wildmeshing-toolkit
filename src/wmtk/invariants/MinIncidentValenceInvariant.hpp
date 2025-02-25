#pragma once

#include <wmtk/attribute/AttributeHandle.hpp>
#include "MeshInvariant.hpp"

namespace wmtk::invariants {
/**
 * Invariant for minimum valence on both incident vertices of an edge.
 */
class MinIncidentValenceInvariant : public Invariant
{
public:
    MinIncidentValenceInvariant(const Mesh& m, long min_valence);
    using Invariant::Invariant;
    bool before(const Simplex& t) const override;
    bool after(PrimitiveType type, const std::vector<Tuple>& t) const override;

private:
    bool is_greater_min_valence(const Tuple& t) const;

    long m_min_valence;
};
} // namespace wmtk::invariants
