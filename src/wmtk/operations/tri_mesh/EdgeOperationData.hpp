#pragma once
#include <array>
#include <vector>
#include <wmtk/Tuple.hpp>

namespace wmtk::operations::tri_mesh {
struct EdgeOperationData
{
    EdgeOperationData() = default;
    EdgeOperationData(const EdgeOperationData&) = default;
    EdgeOperationData(EdgeOperationData&&) = default;

    EdgeOperationData& operator=(const EdgeOperationData&) = default;
    EdgeOperationData& operator=(EdgeOperationData&&) = default;
    //           C
    //         /  \ .
    //    F1  /    \  F2
    //       /      \ .
    //      /        \ .
    //     A----------B
    //      \        /
    //       \      /
    //    F1' \    / F2'
    //         \  /
    //          C'
    // the neighbors are stored in the order of A, B, C, D if they exist
    // vid, ear fid (-1 if it doesn't exit), ear eid

    /**
     * An ear is a face that is adjacent to a face that is incident to the edge on which the
     * operation is performed. In other words, the ears are the neighboring faces to the ones
     * that will be deleted by the operation.
     */
    struct EarData
    {
        long fid = -1; // global fid of the ear, -1 if it doesn't exist
        long eid = -1; // global eid of the ear, -1 if it doesn't exist
    };

    /**
     * Data on the incident face relevant for performing operations.
     */
    struct IncidentFaceData
    {
        // vid of the vertex opposite of the input edge with respect to the input face
        long opposite_vid = -1;
        // the face id from before the operation
        long fid = -1;
        // the fids of the split edge - first one has vertex A, second one has vertex B
        std::array<long, 2> split_f = std::array<long, 2>{{-1, -1}};

        // the copy of edge m_operating_tuple in face(fid)
        Tuple local_operating_tuple;

        // the ear data (i.e FID and EID of the edge/face across the edge.
        // first face/edge include A, second includes B
        std::array<EarData, 2> ears;
    };

    const std::vector<IncidentFaceData>& incident_face_datas() const
    {
        return m_incident_face_datas;
    }

    const std::array<long, 2>& incident_vids() const { return m_spine_vids; }

    long operating_edge_id() const { return m_operating_edge_id; }


    std::array<std::vector<long>, 3> simplex_ids_to_delete;
    std::vector<long> cell_ids_to_update_hash;

    // for multimesh we need to know which global ids are modified to trigger
    // for every simplex dimension (We have 3 in trimesh):
    // a list of [simplex index, {all versions of that simplex}]
    std::vector<std::vector<std::tuple<long, std::vector<Tuple>>>>
        global_simplex_ids_with_potentially_modified_hashes;

    Tuple m_operating_tuple;

    Tuple m_output_tuple; // reference tuple for either operation

    // common simplicies
    std::array<long, 2> m_spine_vids; // V_A_id, V_B_id;
    long spine_eid = -1;
    long m_operating_edge_id = -1;

    // simplices required per-face
    std::vector<IncidentFaceData> m_incident_face_datas;

    std::array<long, 2> split_spine_eids = std::array<long, 2>{{-1, -1}};
    long split_new_vid = -1;
    long split_edge_eid = -1;

    bool is_collapse = false;
};
} // namespace wmtk::operations::tri_mesh
