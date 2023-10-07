
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include <wmtk/multimesh/utils/UpdateEdgeOperationMultiMeshMapFunctor.hpp>
#include <wmtk/utils/mesh_type_from_primitive_type.hpp>


namespace wmtk::operations::utils {
namespace {

struct EdgeSplitFunctor
{
    void operator()(const Mesh&, const Simplex&) const { throw "Unimplemented!"; }
    TriMeshOperationExecutor operator()(const TriMesh& m, const Simplex& s) const
    {
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        TriMeshOperationExecutor exec(mesh, t, hash_accessor);
        exec.split_edge();
        return exec;
    }
    TetMeshOperationExecutor operator()(const TetMesh& m, const Simplex& s) const
    {
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        TetMeshOperationExecutor exec(mesh, t, hash_accessor);
        exec.split_edge();
        return exec;
    }
};


void multi_mesh_split_edge(Mesh& mesh, const Tuple& t);
{
    multimesh::MultiMeshVisitor visitor(
        EdgeSplitFunctor{},
        UpdateEdgeOperationMultiMeshMapFunctor{});
    visitor.execute_from_root(mesh, Simplex(PrimitiveType::Edge, t));
}
