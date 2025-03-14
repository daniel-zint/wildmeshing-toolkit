#pragma once

#include "MeshWriter.hpp"

#include <filesystem>

namespace h5pp {
class File;
}

namespace wmtk {
class HDF5Writer : public MeshWriter
{
public:
    HDF5Writer(const std::filesystem::path& filename);

    void write_top_simplex_type(const PrimitiveType type) override;

    bool write(const int) override { return true; }

    void write_capacities(const std::vector<long>& capacities) override;

    void write(
        const std::string& name,
        const long type,
        const long stride,
        const std::vector<char>& val) override;

    void write(
        const std::string& name,
        const long type,
        const long stride,
        const std::vector<long>& val) override;

    void write(
        const std::string& name,
        const long type,
        const long stride,
        const std::vector<double>& val) override;

    void write(
        const std::string& name,
        const long type,
        const long stride,
        const std::vector<Rational>& val) override;

private:
    std::shared_ptr<h5pp::File> m_hdf5_file;

    template <typename T>
    void write_internal(
        const std::string& name,
        const long type,
        const long stride,
        const std::vector<T>& val);
};

} // namespace wmtk
