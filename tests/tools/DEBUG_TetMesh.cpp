#include "DEBUG_TetMesh.hpp"
#include <catch2/catch_test_macros.hpp>
#include <wmtk/autogen/tet_mesh/autogenerated_tables.hpp>


namespace wmtk::tests_3d {
DEBUG_TetMesh::DEBUG_TetMesh(const TetMesh& m)
    : TetMesh(m)
{}
DEBUG_TetMesh::DEBUG_TetMesh(TetMesh&& m)
    : TetMesh(std::move(m))
{}

bool DEBUG_TetMesh::operator==(const DEBUG_TetMesh& o) const
{
    return static_cast<const TetMesh&>(*this) == static_cast<const TetMesh&>(o);
}

bool DEBUG_TetMesh::operator!=(const DEBUG_TetMesh& o) const
{
    return !(*this == o);
}

void DEBUG_TetMesh::print_state() const {}

auto DEBUG_TetMesh::edge_tuple_between_v1_v2(const long v1, const long v2, const long tid) const
    -> Tuple
{
    ConstAccessor<long> tv = create_accessor<long>(m_tv_handle);
    auto tv_base = create_base_accessor<long>(m_tv_handle);
    Tuple tet = tet_tuple_from_id(tid);
    auto tv0 = tv.const_vector_attribute(tet);
    REQUIRE(tv0 == tv_base.const_vector_attribute(tid));
    long local_vid1 = -1, local_vid2 = -1;
    for (long i = 0; i < tv0.size(); ++i) {
        if (tv0[i] == v1) {
            local_vid1 = i;
        }
        if (tv0[i] == v2) {
            local_vid2 = i;
        }
    }

    long local_eid = -1;
    for (long i = 0; i < 6; ++i) {
        if ((wmtk::autogen::tet_mesh::auto_3d_edges[i][0] == local_vid1 &&
             wmtk::autogen::tet_mesh::auto_3d_edges[i][1] == local_vid2) ||
            (wmtk::autogen::tet_mesh::auto_3d_edges[i][0] == local_vid2 &&
             wmtk::autogen::tet_mesh::auto_3d_edges[i][1] == local_vid1)) {
            local_eid = i;
            break;
        }
    }

    assert(local_eid > -1);

    long local_fid = wmtk::autogen::tet_mesh::auto_3d_table_complete_edge[local_eid][2];
    assert(local_fid > -1);

    return Tuple(local_vid1, local_eid, local_fid, tid, 0);
}

auto DEBUG_TetMesh::edge_tuple_between_v1_v2(
    const long v1,
    const long v2,
    const long v3,
    const long tid) const -> Tuple
{
    ConstAccessor<long> tv = create_accessor<long>(m_tv_handle);
    auto tv_base = create_base_accessor<long>(m_tv_handle);
    Tuple tet = tet_tuple_from_id(tid);
    auto tv0 = tv.const_vector_attribute(tet);
    REQUIRE(tv0 == tv_base.const_vector_attribute(tid));
    long local_vid1 = -1, local_vid2 = -1, local_vid3 = -1;
    for (long i = 0; i < tv0.size(); ++i) {
        if (tv0[i] == v1) {
            local_vid1 = i;
        }
        if (tv0[i] == v2) {
            local_vid2 = i;
        }
        if (tv0[i] == v3) {
            local_vid3 = i;
        }
    }

    long local_eid = -1;
    for (long i = 0; i < 6; ++i) {
        if ((wmtk::autogen::tet_mesh::auto_3d_edges[i][0] == local_vid1 &&
             wmtk::autogen::tet_mesh::auto_3d_edges[i][1] == local_vid2) ||
            (wmtk::autogen::tet_mesh::auto_3d_edges[i][0] == local_vid2 &&
             wmtk::autogen::tet_mesh::auto_3d_edges[i][1] == local_vid1)) {
            local_eid = i;
            break;
        }
    }

    assert(local_eid > -1);

    long local_fid = -1;
    std::array<long, 3> sorted_local_vids = {{local_vid1, local_vid2, local_vid3}};
    std::sort(sorted_local_vids.begin(), sorted_local_vids.end());

    for (long i = 0; i < 4; ++i) {
        if (wmtk::autogen::tet_mesh::auto_3d_faces[i][0] == sorted_local_vids[0] &&
            wmtk::autogen::tet_mesh::auto_3d_faces[i][1] == sorted_local_vids[1] &&
            wmtk::autogen::tet_mesh::auto_3d_faces[i][2] == sorted_local_vids[2]) {
            local_fid = i;
            break;
        }
    }
    assert(local_fid > -1);

    return Tuple(local_vid1, local_eid, local_fid, tid, 0);
}

auto DEBUG_TetMesh::edge_tuple_from_vids(const long v1, const long v2) const -> Tuple
{
    ConstAccessor<long> tv = create_accessor<long>(m_tv_handle);
    auto tv_base = create_base_accessor<long>(m_tv_handle);
    for (long tid = 0; tid < capacity(PrimitiveType::Tetrahedron); ++tid) {
        Tuple tet = tet_tuple_from_id(tid);
        auto tv0 = tv.const_vector_attribute(tet);
        long local_vid1 = -1, local_vid2 = -1;
        for (long i = 0; i < tv0.size(); ++i) {
            if (tv0[i] == v1) {
                local_vid1 = i;
            }
            if (tv0[i] == v2) {
                local_vid2 = i;
            }
        }

        if (local_vid1 == -1 || local_vid2 == -1) {
            // not found
            continue;
        }

        long local_eid = -1;
        for (long i = 0; i < 6; ++i) {
            if ((wmtk::autogen::tet_mesh::auto_3d_edges[i][0] == local_vid1 &&
                 wmtk::autogen::tet_mesh::auto_3d_edges[i][1] == local_vid2) ||
                (wmtk::autogen::tet_mesh::auto_3d_edges[i][0] == local_vid2 &&
                 wmtk::autogen::tet_mesh::auto_3d_edges[i][1] == local_vid1)) {
                local_eid = i;
                break;
            }
        }

        assert(local_eid > -1);

        long local_fid = wmtk::autogen::tet_mesh::auto_3d_table_complete_edge[local_eid][2];
        assert(local_fid > -1);

        long local_vid = wmtk::autogen::tet_mesh::auto_3d_table_complete_edge[local_eid][0];

        return Tuple(local_vid, local_eid, local_fid, tid, 0);
    }
    return Tuple();
}

auto DEBUG_TetMesh::face_tuple_from_vids(const long v1, const long v2, const long v3) const -> Tuple
{
    ConstAccessor<long> tv = create_accessor<long>(m_tv_handle);
    auto tv_base = create_base_accessor<long>(m_tv_handle);
    for (long tid = 0; tid < capacity(PrimitiveType::Tetrahedron); ++tid) {
        Tuple tet = tet_tuple_from_id(tid);
        auto tv0 = tv.const_vector_attribute(tet);
        long local_vid1 = -1, local_vid2 = -1, local_vid3 = -1;
        for (long i = 0; i < tv0.size(); ++i) {
            if (tv0[i] == v1) {
                local_vid1 = i;
            }
            if (tv0[i] == v2) {
                local_vid2 = i;
            }
            if (tv0[i] == v3) {
                local_vid3 = i;
            }
        }

        if (local_vid1 == -1 || local_vid2 == -1 || local_vid3 == -1) {
            continue;
        }

        long local_fid = -1;
        std::array<long, 3> sorted_local_vids = {{local_vid1, local_vid2, local_vid3}};
        std::sort(sorted_local_vids.begin(), sorted_local_vids.end());

        for (long i = 0; i < 4; ++i) {
            if (wmtk::autogen::tet_mesh::auto_3d_faces[i][0] == sorted_local_vids[0] &&
                wmtk::autogen::tet_mesh::auto_3d_faces[i][1] == sorted_local_vids[1] &&
                wmtk::autogen::tet_mesh::auto_3d_faces[i][2] == sorted_local_vids[2]) {
                local_fid = i;
                break;
            }
        }

        assert(local_fid > -1);

        long local_eid = wmtk::autogen::tet_mesh::auto_3d_table_complete_face[local_fid][1];
        assert(local_eid > -1);

        long local_vid = wmtk::autogen::tet_mesh::auto_3d_table_complete_face[local_fid][0];

        return Tuple(local_vid, local_eid, local_fid, tid, 0);
    }
    return Tuple();
}

auto DEBUG_TetMesh::tet_tuple_from_vids(const long v1, const long v2, const long v3, const long v4)
    const -> Tuple
{
    ConstAccessor<long> tv = create_accessor<long>(m_tv_handle);
    auto tv_base = create_base_accessor<long>(m_tv_handle);
    for (long tid = 0; tid < capacity(PrimitiveType::Tetrahedron); ++tid) {
        Tuple tet = tet_tuple_from_id(tid);
        auto tv0 = tv.const_vector_attribute(tet);
        bool find_v1 = false, find_v2 = false, find_v3 = false, find_v4 = false;
        for (long i = 0; i < tv0.size(); ++i) {
            if (tv0[i] == v1) {
                find_v1 = true;
            }
            if (tv0[i] == v2) {
                find_v2 = true;
            }
            if (tv0[i] == v3) {
                find_v3 = true;
            }
            if (tv0[i] == v4) {
                find_v4 = true;
            }
        }

        if (find_v1 && find_v2 && find_v3 && find_v4) {
            return tet;
        }
    }
    return Tuple();
}

const MeshAttributeHandle<long>& DEBUG_TetMesh::t_handle(const PrimitiveType type) const
{
    switch (type) {
    case PrimitiveType::Vertex: return m_tv_handle;
    case PrimitiveType::Edge: return m_te_handle;
    case PrimitiveType::Face: return m_tf_handle;
    case PrimitiveType::Tetrahedron: return m_tt_handle;
    default: throw std::runtime_error("Invalid PrimitiveType");
    }
}

const MeshAttributeHandle<long>& DEBUG_TetMesh::vt_handle() const
{
    return m_vt_handle;
}

const MeshAttributeHandle<long>& DEBUG_TetMesh::et_handle() const
{
    return m_et_handle;
}

const MeshAttributeHandle<long>& DEBUG_TetMesh::ft_handle() const
{
    return m_ft_handle;
}

void DEBUG_TetMesh::reserve_attributes(PrimitiveType type, long size)
{
    Mesh::reserve_attributes(type, size);
}

long DEBUG_TetMesh::id(const Tuple& tuple, PrimitiveType type) const
{
    return TetMesh::id(tuple, type);
}
long DEBUG_TetMesh::id(const Simplex& s) const
{
    return id(s.tuple(), s.primitive_type());
}

Accessor<long> DEBUG_TetMesh::get_cell_hash_accessor()
{
    return TetMesh::get_cell_hash_accessor();
}

auto DEBUG_TetMesh::get_tmoe(const Tuple& t, Accessor<long>& hash_accessor)
    -> TetMeshOperationExecutor
{
    return TetMeshOperationExecutor(*this, t, hash_accessor);
}

long DEBUG_TetMesh::valid_primitive_count(PrimitiveType type) const
{
    long cnt = 0;
    const auto& flag_accessor = get_const_flag_accessor(type);
    for (int i = 0; i < capacity(type); i++) {
        if (flag_accessor.const_scalar_attribute(tuple_from_id(type, i)) != 0) {
            cnt++;
        }
    }
    return cnt;
}


} // namespace wmtk::tests_3d
