//
// Test command to correct vertex list of polygon.
//

#pragma once

#include "util.hpp"

#include <lxsdk/lx_layer.hpp>
#include <lxsdk/lxu_math.hpp>
#include <lxsdk/lxu_vector.hpp>
#include <lxsdk/lxu_command.hpp>

class FaceVisitor : public CLxImpl_AbstractVisitor
{
public:
    LxResult Evaluate()
    {
        unsigned nvert;
        m_poly.VertexCount(&nvert);
        if (nvert <= 3)
            return LXe_OK;;

        LXtID4 type;
        m_poly.Type(&type);
        if ((type != LXiPTYP_FACE) && (type != LXiPTYP_PSUB) && (type != LXiPTYP_SUBD))
            return LXe_OK;

        std::vector<LXtPointID> points;

        if (MeshUtil::PolygonFixedVertexList(m_mesh, m_poly, points))
        {
            m_poly.SetVertexList(points.data(), points.size(), 0);
        }
        return LXe_OK;
    }

    CLxUser_Mesh    m_mesh;
    CLxUser_Polygon m_poly;
};


class CCommand : public CLxBasicCommand
{
public:
    CLxUser_LayerService lyr_S;
    CLxUser_MeshService  msh_S;
    unsigned             select_mode;

    CCommand()
    {
        select_mode = msh_S.SetMode(LXsMARK_SELECT);
    }

    static void initialize()
    {
        CLxGenericPolymorph* srv;

        srv = new CLxPolymorph<CCommand>;
        srv->AddInterface(new CLxIfc_Command<CCommand>);
        srv->AddInterface(new CLxIfc_Attributes<CCommand>);
        srv->AddInterface(new CLxIfc_AttributesUI<CCommand>);
        lx::AddServer("test.correct", srv);
    }

    int basic_CmdFlags()
    {
        return LXfCMD_MODEL | LXfCMD_UNDO;
    }

    void basic_Execute(unsigned int flags)
    {
        CLxUser_LayerScan scan;
        unsigned          n;

        check(lyr_S.BeginScan(LXf_LAYERSCAN_EDIT_POLYS, scan));
        check(scan.Count(&n));

        CLxUser_Mesh        base_mesh;
        CLxUser_Mesh        edit_mesh;

        CLxUser_MeshService mS;
        LXtMarkMode pick = mS.SetMode(LXsMARK_SELECT);

        for (auto i = 0u; i < n; i++)
        {
            check(scan.BaseMeshByIndex(i, base_mesh));
            check(scan.EditMeshByIndex(i, edit_mesh));

            FaceVisitor vis;
            vis.m_mesh = edit_mesh;
            vis.m_poly.fromMesh(edit_mesh);
            vis.m_poly.Enum(&vis, pick);

            scan.SetMeshChange(i, LXf_MESHEDIT_GEOMETRY);
        }

        scan.Apply();
    }
};
