/*
 * Parameterization helper class
 */

#include "parameterize.hpp"
#include "util.hpp"
#include "triangulate.hpp"

#include <lxsdk/lxu_geometry_triangulation.hpp>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_parameterization/LSCM_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/ARAP_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Circular_border_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Square_border_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Barycentric_mapping_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Discrete_authalic_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Discrete_conformal_map_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Mean_value_coordinates_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Fixed_border_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Error_code.h>
#include <CGAL/Surface_mesh_parameterization/parameterize.h>

#include <igl/slim.h>
#include <igl/lscm.h>
#include <igl/flipped_triangles.h>
#include <igl/MappingEnergyType.h>
#include <igl/harmonic.h>

#define MODE_M(c, s) ((LXtMarkMode) (((c) << 16) | s))

#define MARK_ALL	((unsigned int)0xFFFF)
#define MM_CLEAR(m)	(((m) >> 16) & MARK_ALL)
#define MM_SET(m)	((m) & MARK_ALL)
#define APPLY_M(d,c,s)	(((d) & ~(c)) | (s))
#define APPLY_MODE(d,m)	APPLY_M(d,MM_CLEAR(m),MM_SET(m))
#define TEST_M(d,c,s)	((((d) & (s)) == s) && (((d) & (c)) == 0))
#define TEST_MODE(d,m)	TEST_M(d,MM_CLEAR(m),MM_SET(m))


class MarkEdgeVisitor : public CLxImpl_AbstractVisitor
{
public:
    MarkEdgeVisitor () { m_count = 0u; }
    LxResult Evaluate()
    {
        m_edge.SetMarks(MODE_M(m_context->m_mark_seam, 0));
        if (m_context->m_pick && (m_edge.TestMarks(m_context->m_pick) == LXe_TRUE))
            m_count ++;
        return LXe_OK;
    }

    CLxUser_Edge   m_edge;
    unsigned       m_count;
    struct CParam* m_context;
};

class TripleFaceVisitor : public CLxImpl_AbstractVisitor
{
public:
    LxResult Evaluate()
    {
        unsigned nvert;
        m_poly.VertexCount(&nvert);
        if (nvert < 3)
            return LXe_OK;

        m_poly.SetMarks(m_mark_done);

        LXtID4 type;
        m_poly.Type(&type);
        if ((type != LXiPTYP_FACE) && (type != LXiPTYP_PSUB) && (type != LXiPTYP_SUBD))
            return LXe_OK;

        m_context->AddPolygon(m_poly.ID());

        std::vector<LXtPointID> points;

        LXtPointID v0, v1, v2;
        if (nvert == 3)
        {
            m_poly.VertexByIndex(0, &v0);
            m_poly.VertexByIndex(1, &v1);
            m_poly.VertexByIndex(2, &v2);
            m_context->AddTriangle(m_poly.ID(), v0, v1, v2);
        }
        else if (MeshUtil::PolygonFixedVertexList(m_mesh, m_poly, points))
        {
            bool done = false;
            if (nvert > 4)
            {
                LXtVector norm;
                //MeshUtil::VertexListNormal(m_mesh, points, norm);
                m_poly.Normal(norm);
                AxisPlane axisPlane(norm);
                std::vector<std::vector<LXtPointID>> tris;
                CTriangulate ctri(m_mesh);
                LxResult result = ctri.EarClipping(axisPlane, points, tris);
            //    if (std::string(polyTag.Value(LXi_PTAG_MATR)) == "NonAlign")
            //        printf("EarClipping ok (%d) polyID %p nvert = %u\n", (result == LXe_OK), m_poly.ID(), nvert);
                if (result == LXe_OK)
                {
                    for (auto& vert : tris)
                    {
                        m_context->AddTriangle(m_poly.ID(), vert[0], vert[1], vert[2]);
                    }
                    done = true;
                }
            }
            if (!done)
            {
                v0 = points[0];
                for (auto i = 1u; i < points.size()-1; i++)
                {
                    v1 = points[i];
                    v2 = points[i+1];
                    m_context->AddTriangle(m_poly.ID(), v0, v1, v2);
                }
            }
        }
        else
        {
            unsigned count;
            m_poly.GenerateTriangles(&count);
            for (auto i = 0u; i < count; i++)
            {
                m_poly.TriangleByIndex(i, &v0, &v1, &v2);
                m_context->AddTriangle(m_poly.ID(), v0, v1, v2);
            }
        }
        return LXe_OK;
    }

    CLxUser_Mesh    m_mesh;
    CLxUser_Polygon m_poly;
    CLxUser_Point   m_vert;
    CLxUser_MeshMap m_vmap;
    LXtMarkMode     m_mark_done;
    struct CParam*  m_context;
};

class PartFaceVisitor : public CLxImpl_AbstractVisitor
{
public:
    LxResult Evaluate()
    {
    CLxUser_LogService   s_log;
        unsigned nvert;
        m_poly.VertexCount(&nvert);
        if (nvert < 3)
            return LXe_OK;

        LXtID4 type;
        m_poly.Type(&type);
        if ((type != LXiPTYP_FACE) && (type != LXiPTYP_PSUB) && (type != LXiPTYP_SUBD))
            return LXe_OK;

        if (m_poly.TestMarks(m_mark_done) == LXe_TRUE)
            return LXe_OK;

        m_context->m_parts.push_back(std::make_shared<CParam::ParamPart>());
        CParam::ParamPartID part = m_context->m_parts.back();

        part->ix    = 0.0;
        part->iy    = 0.0;
        part->index = static_cast<unsigned>(m_context->m_parts.size() - 1);

        CLxUser_Polygon poly, poly1;
        poly.fromMesh(m_mesh);
        poly1.fromMesh(m_mesh);
        CLxUser_Edge edge;
        edge.fromMesh(m_mesh);

        std::vector<LXtPolygonID> stack;
        LXtPolygonID              pol = m_poly.ID();
        stack.push_back(pol);
        m_poly.SetMarks(m_context->m_mark_done);

        LXtVector axis, norm;
        LXx_VCLR(axis);

        while (!stack.empty())
        {
            pol = stack.back();
            stack.pop_back();
            poly.Select(pol);
            poly.Normal(norm);
            LXx_VADD(axis, norm);
            CParam::ParamFace& face = m_context->m_faces[pol];
            face.part               = part->index;
            for(auto& tri : face.tris)
            {
                tri->part = part->index;
                part->tris.push_back(tri);
            }
            unsigned int nvert = 0u, npol = 0u;
            poly.VertexCount(&nvert);
            for (auto i = 0u; i < nvert; i++)
            {
                LXtPointID v0{}, v1{};
                poly.VertexByIndex(i, &v0);
                poly.VertexByIndex((i + 1) % nvert, &v1);
                edge.SelectEndpoints(v0, v1);
                edge.PolygonCount(&npol);
                if (m_context->IsSeamEdge(edge))
                {
                    //printf("-- skip seam [%u]\n", i);
                    continue;
                }
                for (auto j = 0u; j < npol; j++)
                {
                    LXtPolygonID pol1;
                    edge.PolygonByIndex(j, &pol1);
                    poly1.Select(pol1);
                    if (poly1.TestMarks(m_mark_done) == LXe_TRUE)
                        continue;
                    poly1.SetMarks(m_mark_done);
                    if (poly1.TestMarks(m_context->m_mark_hide) == LXe_TRUE)
                        continue;
                    if (poly1.TestMarks(m_context->m_mark_lock) == LXe_TRUE)
                        continue;
                    if (m_context->m_type == LXiSEL_POLYGON)
                    {
                        if (m_context->m_pick && poly1.TestMarks(m_context->m_pick) == LXe_FALSE)
                            continue;
                    }
                    stack.push_back(pol1);
                }
            }
        }

        printf("** m_triangles %zu part (%u)\n", m_context->m_triangles.size(), part->index);

        return LXe_OK;
    }

    CLxUser_Mesh    m_mesh;
    CLxUser_Polygon m_poly;
    CLxUser_Point   m_vert;
    CLxUser_MeshMap m_vmap;
    LXtMarkMode     m_mark_done;
    struct CParam*  m_context;
};

LxResult CParam::AddPolygon(LXtPolygonID pol)
{
    ParamFace face;
    m_faces[pol] = face;
    return LXe_OK;
}

LxResult CParam::AddTriangle(LXtPolygonID pol, LXtPointID v0, LXtPointID v1, LXtPointID v2)
{
#if 1
    m_triangles.push_back(std::make_shared<ParamTriangle>());
    ParamTriangleID tri = m_triangles.back();
    tri->index = static_cast<int>(m_triangles.size()-1);
#else
    auto tri = new ParamTriangle;
    m_triangles.push_back(tri);
#endif

    LXtVector norm;
    LXtFVector cross;
    if (pol)
    {
        m_poly.Select(pol);
        m_poly.Index(&tri->pol_index);
        m_poly.Normal(norm);
    }
    ParamVerxID dv[3];
    dv[0] = AddVertex(v0, pol, tri);
    dv[1] = AddVertex(v1, pol, tri);
    dv[2] = AddVertex(v2, pol, tri);
#if 0
    printf("AddTriangle pol (%d) relax (%d) vrt %u %u %u dv %u %u %u\n", 
        tri->pol_index, m_relax,
        dv[0]->vrt_index, dv[1]->vrt_index, dv[2]->vrt_index, 
        dv[0]->index, dv[1]->index, dv[2]->index);
#endif

    CLxUser_Edge edge;
    edge.fromMesh(m_mesh);

    for (auto i = 0u; i < 3; i++)
    {
        auto j = (i + 1) % 3;
        edge.SelectEndpoints(dv[i]->vrt, dv[j]->vrt);
        if (IsSeamEdge(edge))
        {
            dv[i]->flags |= DISCO_SEAM;
            dv[j]->flags |= DISCO_SEAM;
        }
    }

    if (pol)
    {
        ParamFace& face = m_faces[pol];
        face.tris.push_back(tri);
    }

    tri->area = MathUtil::AreaTriangle(dv[0]->pos, dv[1]->pos, dv[2]->pos);
    tri->v0   = dv[0];
    tri->v1   = dv[1];
    tri->v2   = dv[2];
    tri->pol  = pol;
    return LXe_OK;
}

LXtPolygonID CParam::TracePolygon(LXtPointID vrt, LXtPolygonID pol, int shift)
{
    CLxUser_Edge edge;
    edge.fromMesh(m_mesh);

    CLxUser_Polygon poly;
    poly.fromMesh(m_mesh);

    LXtPolygonID pol1;
    LXtPointID   vrt1;
    unsigned int npol, nvert;

    m_poly.Select(pol);
    m_poly.VertexCount(&nvert);

    for (auto i = 0u; i < nvert; i++)
    {
        m_poly.VertexByIndex(i, &vrt1);
        if (vrt1 == vrt)
        {
            m_poly.VertexByIndex((i + shift + nvert) % nvert, &vrt1);
            edge.SelectEndpoints(vrt, vrt1);
            if (IsSeamEdge(edge))
                return nullptr;
            edge.PolygonCount(&npol);
            for (auto j = 0u; j < npol; j++)
            {
                edge.PolygonByIndex(j, &pol1);
                poly.Select(pol1);
                if (poly.TestMarks(m_mark_hide) == LXe_TRUE)
                    continue;
                if (poly.TestMarks(m_mark_lock) == LXe_TRUE)
                    continue;
                if (pol1 != pol)
                {
                    return pol1;
                }
            }
            return nullptr;
        }
    }
    return nullptr;
}

// Find and get the ParamTriangle with the given vrt and pol.
CParam::ParamTriangleID CParam::FetchTriangle(LXtPointID vrt, LXtPolygonID pol)
{
    LXtPolygonID pol0 = pol;
    unsigned int npol;

    m_vert.Select(vrt);
    m_vert.PolygonCount(&npol);

    if (m_faces.find(pol) != m_faces.end())
    {
        ParamFace& face = m_faces[pol];
        for (auto& tri : face.tris)
        {
            if (tri->v0->vrt == vrt || tri->v1->vrt == vrt || tri->v2->vrt == vrt)
                return tri;
        }
    }
    npol--;

    pol = pol0;
    while (npol > 0)
    {
        pol = TracePolygon(vrt, pol, +1);
        if (!pol)
            break;
        if (pol == pol0)
            break;
        if (m_faces.find(pol) != m_faces.end())
        {
            ParamFace& face = m_faces[pol];
            for (auto& tri : face.tris)
            {
                if (tri->v0->vrt == vrt || tri->v1->vrt == vrt || tri->v2->vrt == vrt)
                    return tri;
            }
        }
        npol--;
    }

    pol = pol0;
    while (npol > 0)
    {
        pol = TracePolygon(vrt, pol, -1);
        if (!pol)
            break;
        if (pol == pol0)
            break;
        if (m_faces.find(pol) != m_faces.end())
        {
            ParamFace& face = m_faces[pol];
            for (auto& tri : face.tris)
            {
                if (tri->v0->vrt == vrt || tri->v1->vrt == vrt || tri->v2->vrt == vrt)
                    return tri;
            }
        }
        npol--;
    }

    return nullptr;
}

CParam::ParamVerxID CParam::FetchVertex(LXtPointID vrt)
{
    m_vert.Select(vrt);
    unsigned count;
    m_vert.PolygonCount(&count);
    for (auto i = 0u; i < count; i++)
    {
        LXtPolygonID pol;
        m_vert.PolygonByIndex(i, &pol);
        ParamTriangleID ref = FetchTriangle(vrt, pol);
        if (ref)
        {
            if (ref->v0->vrt == vrt)
                return ref->v0;
            if (ref->v1->vrt == vrt)
                return ref->v1;
            if (ref->v2->vrt == vrt)
                return ref->v2;
        }
    }
    return nullptr;
}

CParam::ParamVerxID CParam::AddVertex(LXtPointID vrt, LXtPolygonID pol, ParamTriangleID tri)
{
    if (!pol)
    {
        return FetchVertex(vrt);
    }
    //printf("AddVertex vrt (%p) pol (%p)", vrt, pol);
    ParamTriangleID ref = FetchTriangle(vrt, pol);
    if (ref)
    {
        ParamVerxID dv = nullptr;

        if (ref->v0->vrt == vrt)
            dv = ref->v0;
        if (ref->v1->vrt == vrt)
            dv = ref->v1;
        if (ref->v2->vrt == vrt)
            dv = ref->v2;

        if (dv)
        {
            dv->tris.push_back(tri);
            return dv;
        }
    }

#if 1
    m_vertices.push_back(std::make_shared<ParamVerx>());
    ParamVerxID dv = m_vertices.back();
#else
    ParamVerxID dv = new ParamVerx;
    m_vertices.push_back(dv);
#endif

    dv->tris.push_back(tri);
    dv->vrt   = vrt;
    dv->tri   = tri;
    dv->flags = 0;
    dv->index = static_cast<unsigned>(m_vertices.size()-1);
    dv->marks = LXiMARK_ANY;
    dv->w     = 1.0f;
    m_vert.Select(vrt);
    m_vert.Pos(dv->pos);
    m_vert.Index(&dv->vrt_index);
    return dv;
}

bool CParam::IsSeamEdge(CLxUser_Edge& edge)
{
    if (edge.TestMarks(m_mark_seam) == LXe_TRUE)
        return true;

    unsigned int npol;
    edge.PolygonCount(&npol);
    // edge is boundary
    if (npol == 1)
    {
        edge.SetMarks(m_mark_seam);
        return true;
    }

    if (m_relax)
    {
        if (MeshUtil::IsDiscoEdge(m_mesh, m_vmap, edge))
        {
            edge.SetMarks(m_mark_seam);
            return true;
        }
    }
    else
    {
        // edge is selected
        if (m_pick && edge.TestMarks(m_pick) == LXe_TRUE)
        {
            edge.SetMarks(m_mark_seam);
            return true;
        }
    }
    LXtPolygonID pol{};
    for (auto i = 0u; i < npol; i++)
    {
        edge.PolygonByIndex(i, &pol);
        m_poly.Select(pol);
        if (m_poly.TestMarks(m_mark_hide) == LXe_TRUE)
        {
            edge.SetMarks(m_mark_seam);
            return true;
        }
    }

    return false;
}

LxResult CParam::Setup(LXtMarkMode edge_mark, bool seal, bool relax)
{
    CLxUser_MeshService mesh_svc;
    m_pick = edge_mark;

    // clear edge seam marks
    MarkEdgeVisitor edgeVis;
    edgeVis.m_edge.fromMesh(m_mesh);
    edgeVis.m_context = this;
    edgeVis.m_edge.Enum(&edgeVis, LXiMARK_ANY);

    unsigned count;
    m_mesh.EdgeCount(&count);
    if (m_pick && (count == edgeVis.m_count))
        m_pick = 0;

    m_relax = relax;
    if (relax)
        m_layout = false;

    // generate triangles from surface polygons
    TripleFaceVisitor triFace;
    triFace.m_mesh = m_mesh;
    triFace.m_vmap = m_vmap;
    triFace.m_poly.fromMesh(m_mesh);
    triFace.m_vert.fromMesh(m_mesh);
    triFace.m_mark_done = mesh_svc.ClearMode(LXsMARK_USER_0);
    triFace.m_context = this;
    triFace.m_poly.Enum(&triFace, LXiMARK_ANY);

    printf("Setup seal (%d)\n", seal);
    if (seal)
        SealHoles();

    // divides polygons by seam edges in parts.
    PartFaceVisitor partFace;
    partFace.m_mesh = m_mesh;
    partFace.m_vmap = m_vmap;
    partFace.m_poly.fromMesh(m_mesh);
    partFace.m_vert.fromMesh(m_mesh);
    partFace.m_mark_done = mesh_svc.SetMode(LXsMARK_USER_0);
    partFace.m_context = this;
    partFace.m_poly.Enum(&partFace, LXiMARK_ANY);

    printf("** vertices %zu triangles %zu parts %zu faces %zu\n", m_vertices.size(), m_triangles.size(), m_parts.size(), m_faces.size());
    // finalize part info
    for (auto& dv : m_vertices)
    {
        auto& part = m_parts[dv->tri->part];
        part->box3D.add(dv->pos);
        part->discos.push_back(dv);
        dv->index = part->discos.size() - 1;
    }
    //
    // Add dummpy triangles to seal holes.
    //
    if (seal)
    {
        for (auto& tri : m_triangles)
        {
            if (!tri->pol)
            {
                tri->part = tri->v0->tri->part;
                auto& part = m_parts[tri->part];
                part->tris.push_back(tri);
            }
        }
    }
    for (auto& part : m_parts)
    {
        part->scale  = 1.0;
        part->area3D = 0.0;
        for (auto& tri : m_triangles)
            part->area3D += tri->area;
        part->ox = 0.0;
        part->oy = 0.0;
        SetPart (part);
    }
    return LXe_OK;
}

LxResult CParam::SealHoles()
{
    CLxUser_Edge edge;
    edge.fromMesh(m_mesh);

    std::vector<std::vector<LXtPointID>> loops;
    MeshUtil::MakeBorderLoops(m_mesh, loops);
    printf("SealHoles = %zu\n", loops.size());
    for (auto& loop : loops)
    {
        LXtPointID v0 = loop.front();
        for(auto i = 1u; i < loop.size()-1u; i++)
        {
            LXtPointID v1 = loop.at(i);
            LXtPointID v2 = loop.at(i + 1);
            AddTriangle(nullptr, v0, v1, v2);
        }
    }
    return LXe_OK;
}

void CParam::GetPins(ParamPartID part, int pinn, ParamVerxID* pin1, ParamVerxID* pin2)
{
    LXtVector center, extent;
    LXtVector p1, p2;

    part->box3D.center(center);
    part->box3D.extent(extent);

    LXx_VCPY(p1, center);
    LXx_VCPY(p2, center);
    p1[pinn] += extent[pinn] * 0.5;
    p2[pinn] -= extent[pinn] * 0.5;

    double d1 = extent[pinn];
    double d2 = extent[pinn];

    /*
     * Project onto shortest bbox axis, and lock extra vertices.
     */
    for (auto i = 0u; i < part->discos.size(); i++)
    {
        auto& dv = part->discos[i];
        if (i)
        {
            double d = LXx_VDIST(dv->pos, p1);
            if (d < d1)
            {
                *pin1 = dv;
                d1 = d;
            }
            d = LXx_VDIST(dv->pos, p2);
            if (d < d2)
            {
                *pin2 = dv;
                d2 = d;
            }
        }
        else
        {
            *pin1 = dv;
            *pin2 = dv;
            d1 = LXx_VDIST(dv->pos, p1);
            d2 = LXx_VDIST(dv->pos, p2);
        }
    }
}

//
// Load UV values from the current UV map.
//
void CParam::LoadUVs(ParamPartID part)
{
    for (auto i = 0u; i < part->discos.size(); i++)
    {
        auto& dv = part->discos[i];
        if (m_vmap.test())
        {
            float uv[2];
            m_poly.Select(dv->tri->pol);
            if (LXx_OK(m_poly.MapEvaluate(m_vmap.ID(), dv->vrt, uv)))
            {
                dv->value[0] = uv[0];
                dv->value[1] = uv[1];
            }
        }
    }
    // Update part info using computed UV values.
    SetPart (part);
}

LxResult CParam::Apply(CLxUser_Mesh& edit_mesh, double gaps)
{
    CLxUser_MeshMap vmap;

    vmap.fromMesh(edit_mesh);

    // Get UV map ID for the edit mesh.
    if (LXx_FAIL(vmap.SelectByName(LXi_VMAP_TEXTUREUV, m_name.c_str())))
        vmap.New(LXi_VMAP_TEXTUREUV, m_name.c_str(), &m_vmap_id);
    else
        m_vmap_id = vmap.ID();

    m_poly.fromMesh(edit_mesh);
    m_vert.fromMesh(edit_mesh);

    // Layout computed parts and gets the entier scale.
    double mag = Layout(gaps);

    printf("Apply mag %f\n", mag);
    // Apply computed UVs into UV vertex map.
    for (auto& tri : m_triangles)
        SetUVs(tri, mag);

    return LXe_OK;
}

LxResult CParam::MakeParamMesh(CLxUser_Mesh& edit_mesh)
{
    m_poly.fromMesh(edit_mesh);
    m_vert.fromMesh(edit_mesh);

    std::vector<LXtPointID> new_points;
    new_points.resize(m_vertices.size());

    for(auto i = 0u; i < m_vertices.size(); i++)
    {
        m_vertices[i]->index = i;
        LXtVector pos;
        LXtPointID pntID;
        LXx_VCPY(pos, m_vertices[i]->pos);
        m_vert.New(pos, &pntID);
        new_points[i] = pntID;
        m_vert.Select(m_vertices[i]->vrt);
        m_vert.Remove();
    }

    std::vector<std::vector<LXtPointID>> new_polys;
    for(auto& tri : m_triangles)
    {
        std::vector<LXtPointID> pnts;
        pnts.resize(3);
        pnts[0] = new_points[tri->v0->index];
        pnts[1] = new_points[tri->v1->index];
        pnts[2] = new_points[tri->v2->index];
        LXtPolygonID polID;
        m_poly.New(LXiPTYP_FACE, pnts.data(), 3, 0, &polID);
        if (tri->pol)
        {
            m_poly.Select(tri->pol);
            m_poly.Remove();
        }
        new_polys.push_back(pnts);
    }

    for (auto i = 0u; i < new_polys.size(); i++)
    {
        auto& tri1 = new_polys[i];
        for (auto j = i + 1; j < new_polys.size(); j++)
        {
            auto& tri2 = new_polys[j];
            if ((tri1[0] == tri2[0] && tri1[1] == tri2[1] && tri1[2] == tri2[2]) ||
                (tri1[0] == tri2[1] && tri1[1] == tri2[2] && tri1[2] == tri2[0]) ||
                (tri1[0] == tri2[2] && tri1[1] == tri2[0] && tri1[2] == tri2[1]))
            {
                printf("Flipped triangle tri1 %p %p %p tri2 %p %p %p\n",
                    tri1[0], tri1[1], tri1[2], 
                    tri2[0], tri2[1], tri2[2]);
            }
        }
    }

    return LXe_OK;
}

LxResult CParam::EgenMatrix(ParamPartID part, Eigen::MatrixXd& V, Eigen::MatrixXi& F, Eigen::MatrixXd& V_o, Eigen::VectorXi& b, Eigen::MatrixXd& bc)
{
    unsigned n = 0;

    // V : 3d vertex positions
    V.resize(part->discos.size(), 3);
    n = 0;
    for(auto& disco : part->discos)
    {
        V(n, 0) = disco->pos[0];
        V(n, 1) = disco->pos[1];
        V(n, 2) = disco->pos[2];
        n ++;
    }
    // F : vertex indices of triangles
    F.resize(part->tris.size(), 3);
    n = 0;
    if (part->flipped == part->tris.size())
    {
        for(auto& tri : part->tris)
        {
            F(n, 0) = tri->v0->index;
            F(n, 1) = tri->v2->index;
            F(n, 2) = tri->v1->index;
            n ++;
        }
    }
    else
    {
        for(auto& tri : part->tris)
        {
            F(n, 0) = tri->v0->index;
            F(n, 1) = tri->v1->index;
            F(n, 2) = tri->v2->index;
            n ++;
        }
    }
    // V_o : initial projected UV positions
    V_o.resize(part->discos.size(), 2);
    n = 0;
    for(auto& disco : part->discos)
    {
        V_o(n, 0) = disco->value[0];
        V_o(n, 1) = disco->value[1];
        n ++;
    }
    // b, bc : pinned vertex incides and the positions
    n = 0;
    for(auto& disco : part->discos)
    {
        if (disco->flags & CParam::DISCO_LOCKED)
            n ++;
    }
    printf("LOCKED (%u)\n", n);
    #if 1
    b.resize(n);
    bc.resize(n, 2);
    if (n > 0)
    {
        n = 0;
        for(auto& disco : part->discos)
        {
            if (disco->flags & CParam::DISCO_LOCKED)
            {
                b(n) = disco->index;
                bc(n, 0) = disco->value[0];
                bc(n, 1) = disco->value[1];
            }
        }
    }
    #endif
//    printf("Eigen V (%zu) F (%zu) V_o (%zu) b (%zu) bc (%zu)\n", V.size(), F.size(), V_o.size(), b.size(), bc.size());
//    printf("F cols(%td) rows (%td) V cols(%td) rows (%td)\n", F.cols(), F.rows(), V.cols(), V.rows());
    return LXe_OK;
}

double CParam::Layout(double gaps)
{
    //
    // Reset variables and sort parts by size.
    //
    unsigned ix = 0u;
    unsigned iy = 1u;
    unsigned N  = 0u;
    unsigned i;
    double   mag = 1.0;

    std::sort(m_parts.begin(), m_parts.end(),
        [&](ParamPartID& part1, ParamPartID& part2) { return part1->size > part2->size; });

    ParamPartID part = m_parts[m_parts.size() * 9 / 10];
    i = 8 - (std::log(m_parts.size()) / 0.693);
    if (i > 1)
        m_scale = part->size / i;
    else
        m_scale = part->size;

    //
    // If the base scale size is less than 10 percent of the average size, 
    // take the average size/10.
    //
    double scale = 0.0;
    for (i = 0u; i < m_parts.size(); i++)
    {
        part = m_parts[i];
        scale += part->size;

    }
    scale = scale / static_cast<double>(m_parts.size()) * 0.1;
    if (m_scale < scale)
        m_scale = scale;

    //
    // Set the number of cells per part and compute the total
    // number required.
    //
    LXtVector center, extent;
    LXx_VCLR(center);
    LXx_VCLR(extent);

    auto n = 0u, m = 0u;
    for (i = 0u; i < m_parts.size(); i++)
    {
        part      = m_parts[i];
        part->mag = 1.0;
        part->box2D.extent(extent);
        part->box2D.center(center);
        part->nx  = std::ceil(extent[ix] / m_scale);
        part->ny  = std::ceil(extent[iy] / m_scale);
        n += part->nx * part->ny;

        double dx = m_scale * part->nx;
        double dy = m_scale * part->ny;
        center[ix] = center[ix] + (dx - extent[ix]) * 0.5;
        center[iy] = center[iy] + (dy - extent[iy]) * 0.5;
        extent[ix] = dx;
        extent[iy] = dy;
        LXx_VADDS3(part->box2D._min, center, extent, -0.5);
        LXx_VADDS3(part->box2D._max, center, extent,  0.5);

        part->box2D.extent(extent);
        part->box2D.center(center);
    }

    if (!n)
        return 1.0;

    std::vector<bool> grid;
    m  = n;
    n  = static_cast<unsigned>(std::ceil(std::sqrt(n)));
    iy = 0;
    while (1)
    {
        grid.resize(n * n);
        N = n;

        for (i = 0u; i < n * n; i++)
            grid[i] = false;

        for (i = 0u; i < m_parts.size(); i++)
        {
            if (!FitPart(m_parts[i], N, grid))
                break;
        }

        if (i == m_parts.size())
            break;

        //
        // Choose a grid size increment that will accommodate the remaining area.
        //
        ix = n * n;
        for (i = 0u; i < m_parts.size(); i++)
        {
            part = m_parts[i];
            ix += part->nx * part->ny;
        }
        ix = static_cast<unsigned>(std::ceil(std::sqrt(ix))) - n;
        n += std::max(ix, 1u);
        if (n >= m)
            break;
    }

    if (N > 2)
    {
        i = 1;
        for (n = 0u; n < N - 2u; n++)
        {
            iy = N - 1 - n;
            for (ix = 0; i && ix < N; ix++)
                if (grid[iy * N + ix])
                    i = 0;

            ix = N - 1 - n;
            for (iy = 0; i && iy < N; iy++)
                if (grid[iy * N + ix])
                    i = 0;

            if (!i)
                break;
        }
        mag = N / static_cast<double>(N - n);
    }
    else
        mag = 1.0;

    std::sort(m_parts.begin(), m_parts.end(),
        [&](ParamPartID& part1, ParamPartID& part2) { return part1->index < part2->index; });

    //
    // Set part position and size.
    //
    for (i = 0u; i < m_parts.size(); i++)
    {
        part = m_parts[i];
        if (!m_layout)
        {
            part->x0  = (part->k0 % N + part->nx / 2.0) * (1.0 / N);
            part->y0  = (part->k0 / N + part->ny / 2.0) * (1.0 / N);
        }
        part->wx  = (part->nx * part->nx) / static_cast<double>(N * (part->nx + gaps));
        part->wy  = (part->ny * part->ny) / static_cast<double>(N * (part->ny + gaps));
        part->mag = mag;
    }

    m_layout = true;

    return mag;
}


void CParam::SetPart(ParamPartID part)
{
    double area3D = 0.0;
    double area2D = 0.0;
    part->flipped = 0;
    for(auto& tri : part->tris)
    {
        area3D += tri->area;
        double a = MathUtil::AreaTriangle2D(tri->v0->value[0], tri->v0->value[1], 
                    tri->v1->value[0], tri->v1->value[1], tri->v2->value[0], tri->v2->value[1]);
        area2D += std::abs(a);
        if (a < 0.0)
            part->flipped ++;
    }
    if (area3D > 0.0 && area2D > 0.0)
        part->scale = std::sqrt(area3D / area2D) / m_scale;
    else
        part->scale = 1.0;

    part->area2D = area2D;
    part->area3D = area3D;
    part->box2D.clear();

    LXtVector fv;

    for(auto& dv : part->discos)
    {
        LXx_VSET3(fv, dv->value[0], dv->value[1], 0);
        part->box2D.add(fv);
    }
    LXtVector extent, center;
    part->box2D.extent(extent);
    part->box2D.center(center);
    part->nx   = 0;
    part->ny   = 0;
    part->size = std::max(extent[0], extent[1]);

    part->ox = 0.0;
    part->oy = 0.0;
}

bool CParam::FitPart(ParamPartID part, unsigned N, std::vector<bool>& grid)
{
    unsigned n, k, l, ok;
    unsigned x0, y0, ix, iy;

    n = N * N;
    for (k = 0; k < n; k++)
    {
        if (grid[k])
            continue;

        x0 = k % N;
        y0 = k / N;
        if (x0 + part->nx >= N || y0 + part->ny >= N)
            continue;

        ok = 1;
        for (ix = 0; ok && ix < part->nx; ix++)
            for (iy = 0; ok && iy < part->ny; iy++)
            {
                l = (iy + y0) * N + ix + x0;
                if (grid[l])
                    ok = 0;
            }

        if (!ok)
            continue;

        part->k0 = k;
        for (ix = 0; ok && ix < part->nx; ix++)
            for (iy = 0; ok && iy < part->ny; iy++)
            {
                l       = (iy + y0) * N + ix + x0;
                grid[l] = true;
            }

        return true;
    }
    return false;
}


bool CParam::ConnectPols(ParamVerxID dv, LXtPolygonID pol, const float* value)
{
    unsigned int npol;
    float fv0[2], fv1[2];

    m_poly.Select(pol);
    m_vert.Select(dv->vrt);
    m_vert.PolygonCount(&npol);
    if (LXx_FAIL(m_poly.MapEvaluate(m_vmap_id, dv->vrt, fv0)))
        return false;

    for (auto i = 0u; i < npol; i++)
    {
        LXtPolygonID pol1;
        m_vert.PolygonByIndex(i, &pol1);
        m_poly.Select(pol1);
        if (m_poly.TestMarks(m_pick) == LXe_TRUE)
            continue;
        if (pol1 == pol)
            continue;
        if (m_poly.TestMarks(m_mark_lock) == LXe_TRUE || m_poly.TestMarks(m_mark_hide) == LXe_TRUE)
            continue;
        if (LXx_FAIL(m_poly.MapEvaluate(m_vmap_id, dv->vrt, fv1)))
            continue;
        if (MathUtil::VectorEqual(fv0, fv1, 2))
            m_poly.SetMapValue(dv->vrt, m_vmap_id, value);
    }

    return true;
}

bool CParam::SetUVs(ParamTriangleID tri, double mag)
{
    ParamPartID& part = m_parts[tri->part];
    if (part->locked)
        return false;

    float         x, y;
    float         value0[2], value1[2], value2[2];
    const int     ix = 0, iy = 1;

    LXtVector center, extent;
    part->box2D.center(center);
    part->box2D.extent(extent);

    m_poly.Select(tri->pol);

    if (!m_layout)
    {
        value0[0] = tri->v0->value[0] * m_size_w + part->ox;
        value0[1] = tri->v0->value[1] * m_size_h + part->oy;
        value1[0] = tri->v1->value[0] * m_size_w + part->ox;
        value1[1] = tri->v1->value[1] * m_size_h + part->oy;
        value2[0] = tri->v2->value[0] * m_size_w + part->ox;
        value2[1] = tri->v2->value[1] * m_size_h + part->oy;
        if (m_type == LXiSEL_POLYGON)
        {
            ConnectPols(tri->v0, tri->pol, value0);
            ConnectPols(tri->v1, tri->pol, value1);
            ConnectPols(tri->v2, tri->pol, value2);
        }
        m_poly.SetMapValue(tri->v0->vrt, m_vmap_id, value0);
        m_poly.SetMapValue(tri->v1->vrt, m_vmap_id, value1);
        m_poly.SetMapValue(tri->v2->vrt, m_vmap_id, value2);
    }
    else
    {
        // v0 of triangle
        x         = (tri->v0->value[ix] - center[ix]) / extent[ix];
        y         = (tri->v0->value[iy] - center[iy]) / extent[iy];
        value0[0] = (part->x0 + part->wx * x) * mag * m_size_w + part->ox;
        value0[1] = (part->y0 + part->wy * y) * mag * m_size_h + part->oy;
        m_poly.SetMapValue(tri->v0->vrt, m_vmap_id, value0);

        // v1 of triangle
        x         = (tri->v1->value[ix] - center[ix]) / extent[ix];
        y         = (tri->v1->value[iy] - center[iy]) / extent[iy];
        value1[0] = (part->x0 + part->wx * x) * mag * m_size_w + part->ox;
        value1[1] = (part->y0 + part->wy * y) * mag * m_size_h + part->oy;
        m_poly.SetMapValue(tri->v1->vrt, m_vmap_id, value1);

        // v2 of triangle
        x         = (tri->v2->value[ix] - center[ix]) / extent[ix];
        y         = (tri->v2->value[iy] - center[iy]) / extent[iy];
        value2[0] = (part->x0 + part->wx * x) * mag * m_size_w + part->ox;
        value2[1] = (part->y0 + part->wy * y) * mag * m_size_h + part->oy;
        m_poly.SetMapValue(tri->v2->vrt, m_vmap_id, value2);
    }

    return true;
}

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Point_2 Point_2;
typedef CGAL::Surface_mesh<Point_3> Mesh;

namespace SMP = CGAL::Surface_mesh_parameterization;
namespace PMP = CGAL::Polygon_mesh_processing;

//
// Get the longest halfedge on boundary
//
static Mesh::Halfedge_index GetLongestHalfEdge (Mesh& mesh)
{
    // Get the longest halfedge on the border
    Mesh::Halfedge_index bhd{};
    double longest_length = 0.0;
    for (auto halfedge : mesh.halfedges())
    {
        if (mesh.is_border(halfedge))
        {
            const Point_3& source_point = mesh.point(mesh.source(halfedge));
            const Point_3& target_point = mesh.point(mesh.target(halfedge));
            double length = std::sqrt(CGAL::squared_distance(source_point, target_point));
            if (!bhd.is_valid() || (length > longest_length))
            {
                bhd = halfedge;
                longest_length = length;
            }
        }
    }
    return bhd;
}

//
// Least Squares Conformal Maps
//
LxResult CParam::LSCM(int pinn)
{
    printf("LSCM\n");
    // Parameterize the UVs of each part
    for(auto& part : m_parts)
    {
        ParamVerxID pin1, pin2;

        GetPins(part, pinn, &pin1, &pin2);

        Mesh mesh;

        std::unordered_map<ParamVerxID, Mesh::Vertex_index> index_map;
        std::pair<ParamVerxID, Mesh::Vertex_index> fixed1{};
        std::pair<ParamVerxID, Mesh::Vertex_index> fixed2{};

        // Setup CGAL Surface Mesh
        for (auto& dv : part->discos)
        {
            auto v = mesh.add_vertex(Point_3(dv->pos[0], dv->pos[1], dv->pos[2]));
            index_map.insert(std::make_pair(dv, v));
            if (dv == pin1)
                fixed1 = std::make_pair(dv, v);
            if (dv == pin2)
                fixed2 = std::make_pair(dv, v);
        }
        for (auto& tri : part->tris)
        {
            Mesh::Vertex_index v0 = index_map[tri->v0];
            Mesh::Vertex_index v1 = index_map[tri->v1];
            Mesh::Vertex_index v2 = index_map[tri->v2];
            if (tri->v0->vrt_index == 773 || tri->v1->vrt_index == 773 || tri->v2->vrt_index == 773)
            printf("add face (%u) %u %u %u\n", tri->index, tri->v0->vrt_index, tri->v1->vrt_index, tri->v2->vrt_index);
            mesh.add_face(v0, v1, v2);
        }
        printf("-- part discos = %zu tris = %zu\n", part->discos.size(), part->tris.size());
        printf("   fixed1 %u fixed2 %u\n", fixed1.first->vrt_index, fixed2.first->vrt_index);

        // Get the longest halfedge on the border
        Mesh::Halfedge_index bhd = GetLongestHalfEdge (mesh);
        if (!bhd.is_valid())
        {
            printf("LSCM No border edge\n");
            part->locked = true;
	        continue;
        }

        // Pin the vertices at leaset two.
        auto uv_map = mesh.add_property_map<Mesh::Vertex_index, Point_2>("v:uv").first;

        // Create vertex index map and parameterized map
        typedef boost::property_map<Mesh, boost::vertex_index_t>::const_type VertexIndexMap;
        VertexIndexMap vertex_index_map = get(boost::vertex_index, mesh);

        typedef boost::vector_property_map<bool, VertexIndexMap> VertexParameterizedMap;
        VertexParameterizedMap vertex_param_map(vertex_index_map);
 
        typedef SMP::Two_vertices_parameterizer_3<Mesh>                Border_parameterizer;
        typedef SMP::LSCM_parameterizer_3<Mesh, Border_parameterizer>  Parameterizer;

        // LSCM Parameterization
        Parameterizer parameterizer(Border_parameterizer(fixed1.second, fixed2.second));
        auto status = SMP::OK;
        try
        {
            status = parameterizer.parameterize (mesh, bhd, uv_map, vertex_index_map, vertex_param_map);
            printf("LSCM status (%d)\n", status == SMP::OK);
        }
        catch (const CGAL::Assertion_exception& e)
        {
            std::cerr << "CGAL Assertion Exception: " << e.what() << std::endl;
	        return LXe_FAILED;
        }
        catch (const std::exception& e)
        {
            printf("Standard Exception: %s\n", e.what());
	        return LXe_FAILED;
        }
        catch (...)
        {
            std::cerr << "Unknown exception occurred!" << std::endl;
	        return LXe_FAILED;
        }
        
        if (status != SMP::OK)
        {
	        return LXe_FAILED;
        }

        for (const auto& v : mesh.vertices()) {
            const Point_2& uv = uv_map[v];
            auto& dv = part->discos[v];
            dv->value[0] = uv.x();
            dv->value[1] = uv.y();
        }
        
        // Update part info using computed UV values.
        SetPart (part);
    }
	return LXe_OK;
}

//
// As Rigid As Possible Parameterization
//
LxResult CParam::ARAP()
{
    printf("ARAP\n");
    // Parameterize the UVs of each part
    for(auto& part : m_parts)
    {
        Mesh mesh;

        std::unordered_map<ParamVerxID, Mesh::Vertex_index> index_map;

        // Setup CGAL Surface Mesh
        for (auto& dv : part->discos)
        {
            auto v = mesh.add_vertex(Point_3(dv->pos[0], dv->pos[1], dv->pos[2]));
            index_map.insert(std::make_pair(dv, v));
        }
        for (auto& tri : part->tris)
        {
            Mesh::Vertex_index v0 = index_map[tri->v0];
            Mesh::Vertex_index v1 = index_map[tri->v1];
            Mesh::Vertex_index v2 = index_map[tri->v2];
            mesh.add_face(v0, v1, v2);
        }

        // Get the longest halfedge on the border
        Mesh::Halfedge_index bhd = GetLongestHalfEdge (mesh);
        if (!bhd.is_valid())
        {
            printf("ARAP No border edge\n");
            part->locked = true;
	        continue;
        }

        // Pin the vertices at leaset two.
        auto uv_map = mesh.add_property_map<Mesh::Vertex_index, Point_2>("v:uv").first;

        // Create vertex index map and parameterized map
        typedef boost::property_map<Mesh, boost::vertex_index_t>::const_type VertexIndexMap;
        VertexIndexMap vertex_index_map = get(boost::vertex_index, mesh);

        typedef boost::vector_property_map<bool, VertexIndexMap> VertexParameterizedMap;
        VertexParameterizedMap vertex_param_map(vertex_index_map);
 
        typedef SMP::ARAP_parameterizer_3<Mesh>  Parameterizer;

        // ARAP Parameterization
        Parameterizer parameterizer;
        auto status = SMP::OK;
        try
        {
            status = parameterizer.parameterize (mesh, bhd, uv_map, vertex_index_map, vertex_param_map);
            printf("ARAP status (%d)\n", status == SMP::OK);
        }
        catch (const CGAL::Assertion_exception& e)
        {
            s_log.DebugOut(LXi_DBLOG_NORMAL, "CGAL Assertion : %s", e.what());
	        return LXe_FAILED;
        }
        catch (const std::exception& e)
        {
            s_log.DebugOut(LXi_DBLOG_NORMAL, "Standard : %s", e.what());
	        return LXe_FAILED;
        }
        catch (...)
        {
            s_log.DebugOut(LXi_DBLOG_NORMAL, "Unknown exception occurred!");
	        return LXe_FAILED;
        }
        
        if (status != SMP::OK)
        {
	        return LXe_FAILED;
        }

        for (const auto& v : mesh.vertices()) {
            const Point_2& uv = uv_map[v];
            auto& dv = part->discos[v];
            dv->value[0] = uv.x();
            dv->value[1] = uv.y();
        }
        
        // Update part info using computed UV values.
        SetPart (part);
    }
	return LXe_OK;
}

//
// Fixed Border Prameterizations
//
LxResult CParam::Border(unsigned border, unsigned param)
{
    printf("BORDER\n");
    // Parameterize the UVs of each part
    for(auto& part : m_parts)
    {
        Mesh mesh;

        std::unordered_map<ParamVerxID, Mesh::Vertex_index> index_map;

        // Setup CGAL Surface Mesh
        for (auto& dv : part->discos)
        {
            auto v = mesh.add_vertex(Point_3(dv->pos[0], dv->pos[1], dv->pos[2]));
            index_map.insert(std::make_pair(dv, v));
        }
        for (auto& tri : part->tris)
        {
            Mesh::Vertex_index v0 = index_map[tri->v0];
            Mesh::Vertex_index v1 = index_map[tri->v1];
            Mesh::Vertex_index v2 = index_map[tri->v2];
            mesh.add_face(v0, v1, v2);
        }

        // Get the longest halfedge on the border
        Mesh::Halfedge_index bhd = GetLongestHalfEdge (mesh);
        if (!bhd.is_valid())
        {
            printf("ARAP No border edge\n");
            part->locked = true;
	        continue;
        }

        // Pin the vertices at leaset two.
        auto uv_map = mesh.add_property_map<Mesh::Vertex_index, Point_2>("v:uv").first;

        // Create vertex index map and parameterized map
        typedef boost::property_map<Mesh, boost::vertex_index_t>::const_type VertexIndexMap;
        VertexIndexMap vertex_index_map = get(boost::vertex_index, mesh);

        typedef boost::vector_property_map<bool, VertexIndexMap> VertexParameterizedMap;
        VertexParameterizedMap vertex_param_map(vertex_index_map);
 
        typedef SMP::Square_border_uniform_parameterizer_3<Mesh> Square_parameterizer;
        typedef SMP::Circular_border_uniform_parameterizer_3<Mesh> Circular_parameterizer;

        SMP::Error_code status = SMP::OK;

        try
        {
            // Border Parameterization
            if (border == BORDER_SQUARE)
            {
                switch (param)
                {
                    case PARAM_DISCRETE_CONFORMAL:
                        {
                        SMP::Discrete_conformal_map_parameterizer_3<Mesh, Square_parameterizer> parameterizer;
                        status = parameterizer.parameterize (mesh, bhd, uv_map, vertex_index_map, vertex_param_map);
                        }
                        break;
                    case PARAM_DISCRETE_AUTHANLIC:
                        {
                        SMP::Discrete_authalic_parameterizer_3<Mesh, Square_parameterizer> parameterizer;
                        status = parameterizer.parameterize (mesh, bhd, uv_map, vertex_index_map, vertex_param_map);
                        }
                        break;
                    case PARAM_MVC:
                        {
                        SMP::Mean_value_coordinates_parameterizer_3<Mesh, Square_parameterizer> parameterizer;
                        status = parameterizer.parameterize (mesh, bhd, uv_map, vertex_index_map, vertex_param_map);
                        break;
                        }
                    case PARAM_BARYCENTRIC:
                        {
                        SMP::Barycentric_mapping_parameterizer_3<Mesh, Square_parameterizer> parameterizer;
                        status = parameterizer.parameterize (mesh, bhd, uv_map, vertex_index_map, vertex_param_map);
                        }
                        break;
                }
            }
            else if (border == BORDER_CIRCULAR)
            {
                switch (param)
                {
                    case PARAM_DISCRETE_CONFORMAL:
                        {
                        SMP::Discrete_conformal_map_parameterizer_3<Mesh, Circular_parameterizer> parameterizer;
                        status = parameterizer.parameterize (mesh, bhd, uv_map, vertex_index_map, vertex_param_map);
                        }
                        break;
                    case PARAM_DISCRETE_AUTHANLIC:
                        {
                        SMP::Discrete_authalic_parameterizer_3<Mesh, Circular_parameterizer> parameterizer;
                        status = parameterizer.parameterize (mesh, bhd, uv_map, vertex_index_map, vertex_param_map);
                        }
                        break;
                    case PARAM_MVC:
                        {
                        SMP::Mean_value_coordinates_parameterizer_3<Mesh, Circular_parameterizer> parameterizer;
                        status = parameterizer.parameterize (mesh, bhd, uv_map, vertex_index_map, vertex_param_map);
                        }
                        break;
                    case PARAM_BARYCENTRIC:
                        {
                        SMP::Barycentric_mapping_parameterizer_3<Mesh, Circular_parameterizer> parameterizer;
                        status = parameterizer.parameterize (mesh, bhd, uv_map, vertex_index_map, vertex_param_map);
                        }
                        break;
                }
            }
        }
        catch (const CGAL::Assertion_exception& e)
        {
            printf("CGAL Assertion Exception: %s\n", e.what());
	        return LXe_FAILED;
        }
        catch (const std::exception& e)
        {
            printf("Standard Exception: %s\n", e.what());
	        return LXe_FAILED;
        }
        catch (...)
        {
            printf("Unknown exception occurred!\n");
	        return LXe_FAILED;
        }
        
        if (status != SMP::OK)
        {
	        return LXe_FAILED;
        }

        for (const auto& v : mesh.vertices()) {
            const Point_2& uv = uv_map[v];
            auto& dv = part->discos[v];
            // Flipped UVs with Square mode. (CGAL Bug?)
            if (border == BORDER_SQUARE)
                dv->value[0] = 1.0 - uv.x();
            else
                dv->value[0] = uv.x();
            dv->value[1] = uv.y();
        }
        
        // Update part info using computed UV values.
        SetPart (part);
    }
	return LXe_OK;
}


//
// CGAL Default Prameterization, namly Floater Mean Value Coordinates
//
LxResult CParam::FMVC()
{
    printf("FMVC\n");
    // Parameterize the UVs of each part
    for(auto& part : m_parts)
    {
        Mesh mesh;

        std::unordered_map<ParamVerxID, Mesh::Vertex_index> index_map;

        // Setup CGAL Surface Mesh
        for (auto& dv : part->discos)
        {
            auto v = mesh.add_vertex(Point_3(dv->pos[0], dv->pos[1], dv->pos[2]));
            index_map.insert(std::make_pair(dv, v));
        }
        for (auto& tri : part->tris)
        {
            Mesh::Vertex_index v0 = index_map[tri->v0];
            Mesh::Vertex_index v1 = index_map[tri->v1];
            Mesh::Vertex_index v2 = index_map[tri->v2];
            mesh.add_face(v0, v1, v2);
        }

        // Get the longest halfedge on the border
        Mesh::Halfedge_index bhd = GetLongestHalfEdge (mesh);
        if (!bhd.is_valid())
        {
            printf("FMVC No border edge\n");
            part->locked = true;
	        continue;
        }

        // Added UV map to the surface mesh.
        auto uv_map = mesh.add_property_map<Mesh::Vertex_index, Point_2>("v:uv").first;

        // Floater Mean Value Coordinates Parameterization
        auto status = SMP::OK;
        try
        {
            status = SMP::parameterize (mesh, bhd, uv_map);
            printf("FMVC status (%d)\n", status == SMP::OK);
        }
        catch (const CGAL::Assertion_exception& e)
        {
            s_log.DebugOut(LXi_DBLOG_NORMAL, "CGAL Assertion : %s", e.what());
	        return LXe_FAILED;
        }
        catch (const std::exception& e)
        {
            s_log.DebugOut(LXi_DBLOG_NORMAL, "Standard : %s", e.what());
	        return LXe_FAILED;
        }
        catch (...)
        {
            s_log.DebugOut(LXi_DBLOG_NORMAL, "Unknown exception occurred!");
	        return LXe_FAILED;
        }
        
        if (status != SMP::OK)
        {
	        return LXe_FAILED;
        }

        for (const auto& v : mesh.vertices()) {
            const Point_2& uv = uv_map[v];
            auto& dv = part->discos[v];
            dv->value[0] = uv.x();
            dv->value[1] = uv.y();
        }
        
        // Update part info using computed UV values.
        SetPart (part);
    }
	return LXe_OK;
}

LxResult CParam::SLIM(int iter)
{
    // Precompute initial UVs using FMVC method.
    if (m_relax == false)
    {
        LxResult result = FMVC();
        if (result != LXe_OK)
            return result;
    }

    for(auto& part : m_parts)
    {
        // Load UVs from the current map for the relax mode.
        if (m_relax == true)
        {
            LoadUVs(part);
        }

        igl::SLIMData slim_data;

        printf("SLIM iter (%d)\n", iter);
        // Set vertex positions, triangle indices, initial UVs, locked UVs and the indices into
        // Eigen matrices.
        EgenMatrix(part, slim_data.V, slim_data.F, slim_data.V_o, slim_data.b, slim_data.bc);

        // Pre-compute slim data
        slim_data.slim_energy = igl::SYMMETRIC_DIRICHLET;
        slim_data.soft_const_p = std::numeric_limits<double>::max();
        Eigen::VectorXi b;
        Eigen::MatrixXd bc;

        printf("flipped_triangles (%zu)\n", igl::flipped_triangles(slim_data.V_o,slim_data.F).size());
 //     igl::harmonic(slim_data.V,slim_data.F,b,bc,1,slim_data.V_o);
 //     if (igl::flipped_triangles(slim_data.V_o,slim_data.F).size() != 0) {
 //         igl::harmonic(slim_data.F,b,bc,1,slim_data.V_o); // use uniform laplacian
 //     }

        slim_precompute(slim_data.V,
                      slim_data.F,
                      slim_data.V_o,
                      slim_data,
                      slim_data.slim_energy,
                      b,
                      bc,
                      slim_data.soft_const_p);
#if 0
    printf("** slim_data\n");
    printf("   V %zu\n", slim_data.V.size());
    printf("   F %zu\n", slim_data.F.size());
    printf("   slim_energy %u\n", slim_data.slim_energy);
    printf("   b %zu\n", slim_data.b.size());
    printf("   bc %zu\n", slim_data.bc.size());
    printf("   exp_factor %f\n", slim_data.exp_factor);
    printf("   mesh_improvement_3d %d\n", slim_data.mesh_improvement_3d);
    printf("   first_solve %d\n", slim_data.first_solve);
    printf("   has_pre_calc %d\n", slim_data.has_pre_calc);
    printf("   mesh_area %f\n", slim_data.mesh_area);
    printf("   avg_edge_length %f\n", slim_data.avg_edge_length);
#endif

        // Execute slim solver
        try
        {
            slim_solve(slim_data, iter);
        }
        catch (const std::exception& e)
        {
            s_log.DebugOut(LXi_DBLOG_NORMAL, "Exception: %s", e.what());
            printf("Standard Exception: %s\n", e.what());
	        return LXe_FAILED;
        }
        catch (...)
        {
            printf("Unknown exception occurred!\n");
	        return LXe_FAILED;
        }

        // Push back the unwrapped UVs to part.
        unsigned n = 0;
        for(auto& disco : part->discos)
        {
            disco->value[0] = slim_data.V_o(n, 0);
            disco->value[1] = slim_data.V_o(n, 1);
            n ++;
        }
        
        // Update part info using computed UV values.
        SetPart (part);
    }
	return LXe_OK;
}



