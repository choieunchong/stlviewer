#pragma once
#include "OpenMesh/Core/Mesh/TriMeshT.hh"
#include "OpenMesh/Core/Mesh/Traits.hh"
#include "OpenMesh/Core/IO/MeshIO.hh"
#include "OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh"
#include "OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh"
 

struct MeshTraits : public OpenMesh::DefaultTraits
{
    typedef OpenMesh::Vec3d Point;
    typedef OpenMesh::Vec3d Normal;
    typedef OpenMesh::Vec4f Color; // 0.0F ~ 1.0F RGB + Alpha (4D-Vector)

    VertexAttributes(
        OpenMesh::Attributes::Status |
        OpenMesh::Attributes::Normal |
        OpenMesh::Attributes::Color);

    EdgeAttributes(
        OpenMesh::Attributes::Status);

    FaceAttributes(
        OpenMesh::Attributes::Status |
        OpenMesh::Attributes::Normal |
        OpenMesh::Attributes::Color);

};

typedef OpenMesh::TriMesh_ArrayKernelT<MeshTraits>  TriMesh;