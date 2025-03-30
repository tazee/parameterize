//
// Triangulate class
//
#pragma once

#include <lxsdk/lx_visitor.hpp>
#include <lxsdk/lx_vmodel.hpp>

#include <lxsdk/lx_mesh.hpp>
#include <lxsdk/lxidef.h>
#include <lxsdk/lxu_math.hpp>
#include <lxsdk/lxu_vector.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_vertex_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>

#include <CGAL/mark_domain_in_triangulation.h>

#include <vector>

#include "util.hpp"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_2<K> Vb;
typedef CGAL::Delaunay_mesh_face_base_2<K> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds> CDT;
typedef CGAL::Delaunay_mesh_size_criteria_2<CDT> Criteria;
typedef CGAL::Delaunay_mesher_2<CDT, Criteria> Mesher;

typedef CDT::Vertex_handle Vertex_handle;
typedef CDT::Face_handle Face_handle;
typedef CDT::Point CPoint;

class CTriangulate
{
public:
    CTriangulate (CLxUser_Mesh& mesh) { m_mesh = mesh; }
    LxResult ConstraintDelaunay(AxisPlane& axisPlane, std::vector<LXtPointID>& source, std::vector<std::vector<LXtPointID>>& tris)
    {
        CLxUser_Point point, point1;
        point.fromMesh(m_mesh);
        point1.fromMesh(m_mesh);

        CDT cdt;

        std::vector<Vertex_handle> vertex_handles;

        std::unordered_map<LXtPointID,unsigned> indices;

        CLxUser_MeshService s_mesh;

        LXtMarkMode mark_dupl;
        mark_dupl = s_mesh.ClearMode(LXsMARK_USER_0);

        auto nvert = source.size();
        for (auto i = 0u; i < nvert; i++)
        {
            point.Select(source[i]);
            point.SetMarks(mark_dupl);
        }

        mark_dupl = s_mesh.SetMode(LXsMARK_USER_0);

        unsigned n = 0;
        for (auto i = 0u; i < nvert; i++)
        {
            LXtPointID vrt = source[i];
            if (indices.find(vrt) == indices.end())
            {
                indices.insert(std::make_pair(vrt, n ++));
            }
            else
            {
                point.Select(vrt);
                point.SetMarks(mark_dupl);
            }
        }

        double   z_ave = 0.0;
        for (auto i = 0u; i < nvert; i++)
        {
            point.Select(source[i]);
            LXtFVector pos;
            point.Pos(pos);
            double x, y, z;
            axisPlane.ToPlane(pos, x, y, z);
            try
            {
                vertex_handles.push_back(cdt.insert(CPoint(x, y)));
            }
            catch(...)
            {
                printf("CGAL Error pos %f %f %f x %f y %f line (%d)\n", pos[0], pos[1], pos[2], x, y, __LINE__);
                return LXe_FAILED;
            }
            z_ave += z;
        }

        // averaged z value on axis plane
        z_ave /= static_cast<double>(source.size());

        // Set vertex projected positions and edge links.
        for (auto i = 0u; i < nvert; i++)
        {
            LXtPointID vrt, vrt1;
            vrt  = source[i];
            vrt1 = source[(i + 1) % nvert];
            point.Select(vrt);
            point1.Select(vrt1);
            if (MeshUtil::IsKeyholeBridge(point, point1, source))
            {
                continue;
            }
            int v1 = indices[vrt];
            int v2 = indices[vrt1];
            try
            {
                cdt.insert_constraint(vertex_handles[v1], vertex_handles[v2]);
            }
            catch(...)
            {
                printf("CGAL Error v1 (%d) v2 (%d) line (%d)\n", v1, v2, __LINE__);
                return LXe_FAILED;
            }
        }

        if (!cdt.is_valid())
        {
            return LXe_FAILED;
        }
        
        std::unordered_map<Face_handle, bool> in_domain_map;
        boost::associative_property_map< std::unordered_map<Face_handle,bool> > in_domain(in_domain_map);
    
        // Mark facets that are inside the domain bounded by the polygon
        try
        {
            CGAL::mark_domain_in_triangulation(cdt, in_domain);
        }
        catch(...)
        {
            printf("CGAL Error line (%d)\n", __LINE__);
            return LXe_FAILED;
        }

        std::vector<LXtPointID> vert(3);

        tris.clear();

        // Make a map to get index from vertex handle.
        std::unordered_map<Vertex_handle,int> vertex_to_index;
        int index = 0;
        for (auto vertex = cdt.finite_vertices_begin(); vertex != cdt.finite_vertices_end(); vertex++)
        {
            vertex_to_index[vertex] = index++;
        }
        assert(static_cast<size_t>(index) == source.size());

        // Make triangle face polygons into the edit mesh.
        for (auto face = cdt.finite_faces_begin(); face != cdt.finite_faces_end(); face++)
        {
            // Skip if the face is not in domain.
            if (!get(in_domain,face))
                continue;

            // Get three vertices of the triangle
            for (auto i = 0; i < 3; i++)
            {
                Vertex_handle vh = face->vertex(i);
                index = vertex_to_index[vh];
                vert[i] = source[index];
            }
            // Store a new triangle vertices
            tris.push_back(vert);
        }

        return LXe_OK;
    }

    // Triangulate the polygon by ear clipping method.
    // This method is known as ear clipping and sometimes ear trimming. An efficient algorithm for 
    // cutting off ears was discovered by Hossam ElGindy, Hazel Everett, and Godfried Toussaint.
    LxResult EarClipping(AxisPlane& axisPlane, std::vector<LXtPointID>& source, std::vector<std::vector<LXtPointID>>& tris)
    {
        if (source.size() < 3)
            return LXe_FAILED;

        if (source.size() == 3)
        {
            tris.push_back(source);
            return LXe_OK;
        }

        CLxUser_Point point, point1;
        point.fromMesh(m_mesh);
        point1.fromMesh(m_mesh);

        auto orient = MeshUtil::VertexListOrientation(m_mesh, axisPlane, source);
        LXtVector n0, n1;
        MeshUtil::TriangleNormal(m_mesh, source.back(), source.front(), source[1], n0);
        bool flip = false;
        bool done = false;

        std::vector<LXtPointID> verts = source;

        while (verts.size() > 3)
        {
            LXtPointID vp = verts.back();
            LXtPointID vc = verts.front();
            LXtPointID vn = verts[1];
            auto index = 0u;
            for (auto i = 0u; i < verts.size(); i++)
            {
                vn = vp;
                vp = vc;
                vc = verts[(i + 1) % verts.size()];
                if (MeshUtil::TriangleOriented(m_mesh, axisPlane, verts, orient, vn, vp, vc))
                {
                    index = i;
                    break;
                }
            }
            if (!done)
            {
                MeshUtil::TriangleNormal(m_mesh, vp, vc, vn, n1);
                flip = (LXx_VDOT(n0, n1) < 0.0);
                done = true;
            }
            if (flip)
                tris.push_back({vp, vn, vc});
            else
                tris.push_back({vp, vc, vn});

            verts.erase(verts.begin() + index);
        }
        if (flip)
            tris.push_back({verts[0], verts[2], verts[1]});
        else
            tris.push_back({verts[0], verts[1], verts[2]});

        return LXe_OK;
    }

    CLxUser_Mesh m_mesh;
};