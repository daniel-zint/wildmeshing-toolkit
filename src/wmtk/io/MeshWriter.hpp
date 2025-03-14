#pragma once

#include <wmtk/Primitive.hpp>
#include <wmtk/utils/Rational.hpp>

#include <vector>

namespace wmtk {

class MeshWriter
{
public:
    virtual ~MeshWriter() {}

    virtual bool write(const int dim) = 0;

    virtual void write_top_simplex_type(const PrimitiveType type) = 0;

    virtual void write_capacities(const std::vector<long>& capacities) = 0;

    virtual void write(
        const std::string& name,
        const long type,
        const long stride,
        const std::vector<char>& val) = 0;

    virtual void write(
        const std::string& name,
        const long type,
        const long stride,
        const std::vector<long>& val) = 0;

    virtual void write(
        const std::string& name,
        const long type,
        const long stride,
        const std::vector<double>& val) = 0;

    virtual void write(
        const std::string& name,
        const long type,
        const long stride,
        const std::vector<Rational>& val) = 0;
};

} // namespace wmtk
