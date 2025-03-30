//
// Utility functions for the mesh processing.
//
#pragma once

#include <lxsdk/lx_log.hpp>
#include <lxsdk/lx_mesh.hpp>
#include <lxsdk/lx_value.hpp>
#include <lxsdk/lxu_math.hpp>
#include <lxsdk/lxvmath.h>
#include <lxsdk/lxu_matrix.hpp>
#include <lxsdk/lxu_quaternion.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>

#include <vector>
#include <unordered_set>
#include <tuple>

//
// Basic vector math functions.
//
namespace MathUtil {

    static unsigned MaxExtent(const LXtVector v)
    {
        double a = std::abs(v[0]);
        double b = std::abs(v[1]);
        double c = std::abs(v[2]);
        if (a > b && a > c)
            return 0;
        else if (b >= a && b > c)
            return 1;
        else
            return 2;
    }

    template <typename T>
    static bool VectorEqual(const T* a, const T* b, int n)
    {
        while (n--)
            if (lx::Compare(a[n], b[n]))
                return false;

        return true;
    }

    static double AngleVectors(const LXtVector v0, const LXtVector v1)
    {
        double vlen0, vlen1, x;

        vlen0 = LXx_VLEN(v0);
        vlen1 = LXx_VLEN(v1);
        if (vlen0 < lx::Tolerance(vlen0) || vlen1 < lx::Tolerance(vlen1))
            return 0.0;
        x = LXx_VDOT(v0, v1) / vlen0 / vlen1;
        x = LXxCLAMP(x, -1.0, 1.0);
        return std::acos(x);
    }

    static void VectorRotation(LXtMatrix m, const LXtVector v0, const LXtVector v1)
    {
        LXtVector vo;
        double    qq[4];

        if (MathUtil::VectorEqual(v0, v1, LXdND))
        {
            lx::MatrixIdent(m);
            return;
        }

        LXx_VCROSS(vo, v1, v0);
        double theta = AngleVectors(v1, v0);
        if (std::abs(theta) < lx::Tolerance(theta))
        {
            lx::MatrixIdent(m);
            return;
        }
        if (!lx::VectorNormalize(vo))
        {
            qq[0] = 0.0;
            qq[1] = std::sin(theta / 2);
            qq[2] = 0.0;
            qq[3] = std::cos(theta / 2);
        }
        else
        {
            double sint = sin(theta / 2);
            qq[0]       = sint * vo[0];
            qq[1]       = sint * vo[1];
            qq[2]       = sint * vo[2];
            qq[3]       = std::cos(theta / 2);
        }
        CLxQuaternion quat(qq);
        quat.normalize();
        CLxMatrix4 m4 = quat.asMatrix();
        m4.getMatrix3x3(m);
    }

    static double AreaTriangle(const LXtFVector v0, const LXtFVector v1, const LXtFVector v2)
    {
        LXtVector A, B;
        double    AB, AR;

        LXx_VSUB3(A, v1, v0);
        LXx_VSUB3(B, v2, v0);
        AB = LXx_VDOT(A, B);
        AR = LXx_VDOT(A, A) * LXx_VDOT(B, B) - AB * AB;
        if (AR > 0.0)
            return std::sqrt(AR) / 2;
        else
            return 0.0;
    }

    static double AreaTriangle2D(double x0, double y0, double x1, double y1, double x2, double y2)
    {
        double a, b, c, d;

        a = x1 - x0;
        b = y1 - y0;
        c = x2 - x0;
        d = y2 - y0;
        return (a * d - b * c) / 2;
    }

    static double XYAngle(double dx, double dy)
    {
        double a;

        if (dx)
            a = std::atan(dy / dx);
        else if (dy > 0)
            a = LXx_HALFPI;
        else
            a = -LXx_HALFPI;

        if (dx < 0.0)
        {
            if (dy < 0)
                a -= LXx_PI;
            else
                a += LXx_PI;
        }
        return a;
    }

    #define QX 0
    #define QY 1
    #define QZ 2
    #define QW 3

    static void QuaternionNormalize(const LXtQuaternion q, LXtQuaternion qnorm)
    {
        double mag, c;

        mag = (q[QX] * q[QX] + q[QY] * q[QY] + q[QZ] * q[QZ] + q[QW] * q[QW]);
        if (mag > 0)
        {
            c         = 1.0 / std::sqrt(mag);
            qnorm[QX] = q[QX] * c;
            qnorm[QY] = q[QY] * c;
            qnorm[QZ] = q[QZ] * c;
            qnorm[QW] = q[QW] * c;
        }
        else
        {
            qnorm[QW] = 1.0;
            qnorm[QY] = qnorm[QZ] = qnorm[QX] = 0.0;
        }
    }

    static void QuaternionToMatrix(const LXtQuaternion q, LXtMatrix mat)
    {
        double norm, s, xx, yy, zz, xy, xz, yz, wx, wy, wz;

        norm = q[QX] * q[QX] + q[QY] * q[QY] + q[QZ] * q[QZ] + q[QW] * q[QW];
        s    = (norm > lx::Tolerance(norm)) ? 2.0 / norm : 0;

        xx = q[QX] * q[QX] * s;
        yy = q[QY] * q[QY] * s;
        zz = q[QZ] * q[QZ] * s;
        xy = q[QX] * q[QY] * s;
        xz = q[QX] * q[QZ] * s;
        yz = q[QY] * q[QZ] * s;
        wx = q[QW] * q[QX] * s;
        wy = q[QW] * q[QY] * s;
        wz = q[QW] * q[QZ] * s;

        mat[0][0] = 1.0 - (yy + zz);
        mat[0][1] = xy + wz;
        mat[0][2] = xz - wy;

        mat[1][0] = xy - wz;
        mat[1][1] = 1.0 - (xx + zz);
        mat[1][2] = yz + wx;

        mat[2][0] = xz + wy;
        mat[2][1] = yz - wx;
        mat[2][2] = 1.0 - (xx + yy);
    }

    static void MatrixVectorRotation(LXtMatrix m, const LXtVector v0, const LXtVector v1, double frac = 1.0)
    {
        LXtVector     vo;
        LXtQuaternion quat, qout;
        double        theta, sint;

        if (VectorEqual(v0, v1, LXdND))
        {
            lx::MatrixIdent(m);
            return;
        }

        LXx_VCROSS(vo, v1, v0);
        theta = AngleVectors(v1, v0) * frac;
        if (std::fabs(theta) < lx::Tolerance(theta))
        {
            lx::MatrixIdent(m);
            return;
        }
        if (!lx::VectorNormalize(vo))
        {
            quat[0] = 0.0;
            quat[1] = std::sin(theta / 2);
            quat[2] = 0.0;
            quat[3] = std::cos(theta / 2);
        }
        else
        {
            sint    = sin(theta / 2);
            quat[0] = sint * vo[0];
            quat[1] = sint * vo[1];
            quat[2] = sint * vo[2];
            quat[3] = std::cos(theta / 2);
        }
        QuaternionNormalize(quat, qout);
        QuaternionToMatrix(qout, m);
    }

    static void AxisGetViewPlane(unsigned axis, int* ix, int* iy)
    {
        static const int plx[] = { 2, 0, 0 };
        static const int ply[] = { 1, 2, 1 };

        *ix = plx[axis];
        *iy = ply[axis];
    }

    static bool CrossNormal (LXtVector norm, const LXtVector a1, const LXtVector a2, const LXtVector a3)
    {
        LXtVector a, b;

        LXx_VSUB3 (a, a1, a2);
        LXx_VSUB3 (b, a2, a3);
        LXx_VCROSS (norm, a, b);
        return lx::VectorNormalize (norm);
    }

    static bool CrossNormal (LXtFVector norm, const LXtFVector a1, const LXtFVector a2, const LXtFVector a3)
    {
        LXtFVector a, b;

        LXx_VSUB3 (a, a1, a2);
        LXx_VSUB3 (b, a2, a3);
        LXx_VCROSS (norm, a, b);
        return lx::VectorNormalize (norm);
    }
};


//
// Axis plane class to convert the 3D position to 2D position.
//
struct AxisPlane
{
    AxisPlane()
    {
        m_axis = 0;
        m_ix   = 1;
        m_iy   = 2;
        lx::MatrixIdent(m_m);
        lx::MatrixIdent(m_mInv);
    }

    AxisPlane(const LXtVector vec)
    {
        static const unsigned axis0[] = { 1, 2, 0 };
        static const unsigned axis1[] = { 2, 0, 1 };

        m_axis = MathUtil::MaxExtent(vec);
        m_ix   = axis0[m_axis];
        m_iy   = axis1[m_axis];

        LXtVector norm;
        LXx_VUNIT(norm, m_axis);
        lx::MatrixIdent(m_m);
        if (MathUtil::VectorEqual(vec, norm, LXdND) == false)
        {
            MathUtil::VectorRotation(m_m, vec, norm);
        }
        lx::MatrixCopy(m_mInv, m_m);
        lx::MatrixTranspose(m_mInv);
    }

    AxisPlane(unsigned int axis)
    {
        static const unsigned axis0[] = { 1, 2, 0 };
        static const unsigned axis1[] = { 2, 0, 1 };

        m_axis = axis;
        m_ix   = axis0[m_axis];
        m_iy   = axis1[m_axis];

        lx::MatrixIdent(m_m);
        lx::MatrixIdent(m_mInv);
    }

    void ToPlane(const LXtVector pos, double& x, double& y, double& z)
    {
        LXtVector r;
        lx::MatrixMultiply(r, m_m, pos);
        x = r[m_ix];
        y = r[m_iy];
        z = r[m_axis];
    }

    void ToPlane(const LXtFVector pos, double& x, double& y, double& z)
    {
        LXtFVector r;
        lx::MatrixMultiply(r, m_m, pos);
        x = r[m_ix];
        y = r[m_iy];
        z = r[m_axis];
    }

    void FromPlane(LXtVector pos, double x, double y, double z)
    {
        LXtVector r;
        r[m_ix]   = x;
        r[m_iy]   = y;
        r[m_axis] = z;
        lx::MatrixMultiply(pos, m_mInv, r);
    }

    unsigned  m_axis, m_ix, m_iy;
    LXtMatrix m_m, m_mInv;

    double Determ(const LXtFVector prev, const LXtFVector curr, const LXtFVector next)
    {
        double x1, y1, x2, y2, x3, y3, z;
    
        ToPlane(prev, x1, y1, z);
        ToPlane(curr, x2, y2, z);
        ToPlane(next, x3, y3, z);

        double a = (x2 - x1) * (y3 - y1);
        double b = (x3 - x1) * (y2 - y1);
        return a - b;
    }
};


//
// Compute weights at the given position on polygon.
//
struct AxisTriangles
{

    AxisTriangles(CLxUser_Mesh& mesh, CLxUser_Polygon& poly)
    {
        CLxUser_Point point;
        point.fromMesh(mesh);

        LXtVector norm;
        poly.Normal(norm);
        m_axisPlane = AxisPlane(norm);

        std::unordered_map<LXtPointID,unsigned> indices;
        unsigned nvert;
        poly.VertexCount(&nvert);
        for (auto i = 0u; i < nvert; i++)
        {
            LXtPointID vrt;
            poly.VertexByIndex(i, &vrt);
            indices.insert(std::make_pair(vrt, i));
            LXtFVector pos;
            point.Select(vrt);
            point.Pos(pos);
            m_points.push_back(CLxVector(pos));
        }

        unsigned ntri;
        poly.GenerateTriangles(&ntri);
        for (auto i = 0u; i < ntri; i++)
        {
            LXtPointID v0, v1, v2;
            poly.TriangleByIndex(i, &v0, &v1, &v2);
            auto it0 = indices.find(v0);
            auto it1 = indices.find(v1);
            auto it2 = indices.find(v2);
            m_triangles.push_back(std::make_tuple(it0->second, it1->second, it2->second));
        }
    }

    void BaryCentric(CLxVector p0, CLxVector p1, CLxVector p2, double u, double v, double& w0, double& w1, double& w2)
    {
        CLxVector U = p1 - p0;
        CLxVector V = p2 - p0;
        CLxVector T = p2 - p1;
    
        U.normalize();
        V.normalize();
        T.normalize();
    
        double c1 = T.dot(U);
        double s1 = std::sqrt(1.0 - c1 * c1);
    
        T *= -1.0;
        double c2 = T.dot(U);
        double s2 = std::sqrt(1.0 - c2 * c2);
    
        double dt = 0.0;
        double ds = u;
        double dd = std::sqrt(1.0 - ds) * (s1 / s2);
        if (dd)
            dt = v / dd;
    
        w0 = (1.0 - ds) * (1.0 - dt);
        w1 = ds;
        w2 = (1.0 - ds) * dt;
    }

    bool InTriangle(CLxVector pos, CLxVector p0, CLxVector p1, CLxVector p2, double& u, double& v)
    {
        CLxVector vec1(p1 - p0);
        CLxVector vec2(p2 - p0);

        double b[2], c[2], p[2];

        b[0] = vec1[m_axisPlane.m_ix];
        b[1] = vec1[m_axisPlane.m_iy];
        c[0] = vec2[m_axisPlane.m_ix];
        c[1] = vec2[m_axisPlane.m_iy];
        p[0] = pos[m_axisPlane.m_ix] - p0[m_axisPlane.m_ix];
        p[1] = pos[m_axisPlane.m_iy] - p0[m_axisPlane.m_iy];

        u = (p[1] * c[0] - p[0] * c[1]) / (b[1] * c[0] - b[0] * c[1]);
	    v = (p[1] * b[0] - p[0] * b[1]) / (c[1] * b[0] - c[0] * b[1]);
    
        return (u >= 0.0) && (v >= 0.0) && ((u + v) <= 1.0);
    }

    double PointLineSegment (CLxVector& p, CLxVector& x0, CLxVector& x1, double* dist)
    {
        double t, d;

        CLxVector x = CLxVector(x1 - x0);
        CLxVector a = CLxVector(p - x0);
        d = x.dot(x);
        if (!d)
            t = 0.0;
        else
            t = a.dot(x) / d;

        if (dist) {
            if (t < 0.0) {
                *dist = a.length();
            } else if (t > 1.0) {
                a = p - x1;
                *dist = a.length();
            } else {
                x *= t;
                a -= x;
                *dist = a.length();
            }
        }
        return t;
    }

    void MakePositionWeights(const LXtVector pos, std::vector<double>& weights)
    {
        CLxVector p = CLxVector(pos);

        weights.resize(m_points.size(), 0.0);
        for (auto i = 0u; i < weights.size(); i++)
            weights[i] = 0.0;

        for (auto i = 0u; i < m_points.size(); i++)
        {
            auto& p0 = m_points[i];
            if (p0 == p)
            {
                weights[i] = 1.0;
                return;
            }
        }

        for (auto i = 0u; i < m_points.size(); i++)
        {
            auto j = (i + 1) % m_points.size();
            auto& p0 = m_points[i];
            auto& p1 = m_points[j];
            double dist;
            double t = PointLineSegment(p, p0, p1, &dist);
            if ((t >= 0.0) && (t <= 1.0) && (std::abs(dist) <= lx::Tolerance(0.0)))
            {
                weights[j] = t;
                weights[i] = 1.0 - t;
                return;
            }
        }

        for (auto i = 0u; i < m_triangles.size(); i++)
        {
            auto& t  = m_triangles[i];
            auto  i0 = std::get<0>(t);
            auto  i1 = std::get<1>(t);
            auto  i2 = std::get<2>(t);
            auto& p0 = m_points[i0];
            auto& p1 = m_points[i1];
            auto& p2 = m_points[i2];

            double u, v;

            if (InTriangle(p, p0, p1, p2, u, v))
            {
                BaryCentric(p0, p1, p2, u, v, weights[i0], weights[i1], weights[i2]);
                return;
            }
        }
    }

    AxisPlane m_axisPlane;
    std::vector<CLxVector> m_points;
    std::vector<std::tuple<unsigned,unsigned,unsigned>> m_triangles;
};


//
// Polygon and vertex utility functions.
//
namespace MeshUtil {
//
// Get maximum tolerance of the given polygon.
//
static double PolygonTolerance(CLxUser_Polygon& polygon, CLxUser_Point& point)
{
    double m = 0.0;

    unsigned nvert;
    polygon.VertexCount(&nvert);
    for (auto i = 0u; i < nvert; i++)
    {
        LXtPointID vrt;
        LXtFVector pos;
        polygon.VertexByIndex(i, &vrt);
        point.Select(vrt);
        point.Pos(pos);
        for (auto j = 0u; j < LXdND; j++)
        {
            double z = std::abs(pos[j]);
            if (z > m)
                m = z;
        }
    }
    return lx::Tolerance(m);
}

//
// Return true if the vertex pair is appeared twice.
//
static bool IsKeyholeBridge(CLxUser_Point& point, CLxUser_Point& point1, std::vector<LXtPointID>& polygon_vertices)
{
    CLxUser_MeshService s_mesh;
    LXtMarkMode mark_dupl = s_mesh.SetMode(LXsMARK_USER_0);

    if (point.TestMarks(mark_dupl) == LXe_FALSE)
        return false;
    if (point1.TestMarks(mark_dupl) == LXe_FALSE)
        return false;

    LXtPointID vrt0, vrt1;
    vrt0 = point.ID();
    vrt1 = point1.ID();
    unsigned count = 0;

    for (auto i = 0u; i < polygon_vertices.size(); i++)
    {
        auto j = (i + 1) % polygon_vertices.size();
        if ((polygon_vertices[i] == vrt0 && polygon_vertices[j] == vrt1) ||
            (polygon_vertices[i] == vrt1 && polygon_vertices[j] == vrt0))
        {
            count ++;
        }
    }
    return count == 2;
}


//
// Make vertex index table of polygon.
//
static void MakeVertexTable(CLxUser_Polygon& polygon, CLxUser_Point& point, std::vector<LXtPointID>& vertices, std::unordered_map<LXtPointID,unsigned>& indices)
{
    CLxUser_MeshService s_mesh;

    LXtMarkMode mark_dupl;
    mark_dupl = s_mesh.ClearMode(LXsMARK_USER_0);

    unsigned nvert;
    polygon.VertexCount(&nvert);
    for (auto i = 0u; i < nvert; i++)
    {
        LXtPointID vrt;
        polygon.VertexByIndex(i, &vrt);
        point.Select(vrt);
        point.SetMarks(mark_dupl);
    }

    mark_dupl = s_mesh.SetMode(LXsMARK_USER_0);

    unsigned n = 0;
    polygon.VertexCount(&nvert);
    for (auto i = 0u; i < nvert; i++)
    {
        LXtPointID vrt;
        polygon.VertexByIndex(i, &vrt);
        if (indices.find(vrt) == indices.end())
        {
            indices.insert(std::make_pair(vrt, n ++));
            vertices.push_back(vrt);
        }
        else
        {
            point.Select(vrt);
            point.SetMarks(mark_dupl);
        }
    }
}


//
// Compute the area of the loop on axis plane.
//
static double LoopAreaSize(CLxUser_Mesh& mesh, AxisPlane& axisPlane, std::vector<LXtPointID>& loop)
{
    if (loop.size() < 3)
        return 0.0;

    CLxUser_Point point;
    point.fromMesh(mesh);

    LXtFVector pos;
    double x0, y0, z0, x1, y1, z1;
    point.Select(loop[0]);
    point.Pos(pos);
    axisPlane.ToPlane(pos, x0, y0, z0);
    point.Select(loop[1]);
    point.Pos(pos);
    axisPlane.ToPlane(pos, x1, y1, z1);
    double ax, ay, bx, by;
    double S = 0.0;
    ax = x1 - x0;
    ay = y1 - y0;
    for (auto i = 2u; i < loop.size(); i++)
    {
        point.Select(loop[i]);
        point.Pos(pos);
        axisPlane.ToPlane(pos, x1, y1, z1);
        bx = x1 - x0;
        by = y1 - y0;
		S += (ax * by - ay * bx) * 0.5;
		ax = bx;
		ay = by;
    }
    return std::abs(S);
}

//
// Make boundary vertex list and loops from the given polygon.
//
static bool MakeBoundaryVertexList(CLxUser_Mesh& mesh, CLxUser_Polygon& polygon, std::vector<LXtPointID>& vertices, std::vector<std::vector<LXtPointID>>& loops)
{
    CLxUser_Point point0, point1;
    point0.fromMesh(mesh);
    point1.fromMesh(mesh);

    vertices.clear();
    loops.clear();

    std::unordered_map<LXtPointID,unsigned> indices;
    MakeVertexTable(polygon, point0, vertices, indices);
    printf("MakeBoundaryVertexList vertices = %lu indices = %lu\n", vertices.size(), indices.size());

    unsigned nbridge = 0;

    std::vector<std::pair<LXtPointID,LXtPointID>> edges;

    unsigned nvert;
    polygon.VertexCount(&nvert);

    std::vector<LXtPointID> polygon_vertices;

    for (auto i = 0u; i < nvert; i++)
    {
        LXtPointID vrt;
        polygon.VertexByIndex(i, &vrt);
        polygon_vertices.push_back(vrt);
    }

    for (auto i = 0u; i < nvert; i++)
    {
        LXtPointID vrt0, vrt1;
        polygon.VertexByIndex(i, &vrt0);
        polygon.VertexByIndex((i + 1) % nvert, &vrt1);
        point0.Select(vrt0);
        point1.Select(vrt1);
        if (IsKeyholeBridge(point0, point1, polygon_vertices))
        {
            nbridge ++;
        }
        else
        {
            edges.push_back(std::make_pair(vrt0, vrt1));
        }
    }

    //printf("** edges = %lu nbridge = %u\n", edges.size(), nbridge);
    if (!nbridge)
    {
        return false;
    }

    // split the polygon into loops
    std::vector<LXtPointID> loop;
    std::pair<LXtPointID,LXtPointID> edge;

    while (edges.size() > 0)
    {
        if (loop.empty())
        {
            edge = edges.front();
            edges.erase(edges.begin());
            loop.push_back(edge.first);
            printf ("** new loop edges = %lu first (%p) second (%p)\n", edges.size(), edge.first, edge.second);
        }

        bool found = false;
        for(auto it = edges.begin(); it != edges.end();)
        {
            if (it->first == edge.second)
            {
                printf("-- found next edge first (%p) second (%p) loop = %lu edges = %lu\n", it->first, it->second, loop.size(), edges.size());
                loop.push_back(it->first);
                edge = *it;
                if (it->second == loop[0])
                {
                    loops.push_back(loop);
                    loop.clear();
                    printf ("** close loop loops = %lu\n", loops.size());
                }
                it = edges.erase(it);
                found = true;
            }
            else
                it++;
        }
        if (!found)
        {
            printf("** not found loop edges = %lu\n", edges.size());
            return false;
        }
    }

    // find the outer loop
    LXtVector norm;
    polygon.Normal(norm);

    AxisPlane axisPlane(norm);

    double max_area = LoopAreaSize (mesh, axisPlane, loops[0]);
    unsigned index = 0;

    for (auto i = 1u; i < loops.size(); i++)
    {
        double area = LoopAreaSize(mesh, axisPlane, loops[i]);
        if (area > max_area)
        {
            max_area = area;
            index = i;
        }
    }

    vertices = loops[index];
    loops.erase(loops.begin() + index);

    return true;
}

//
// Return true if the polygon is opened.
//
static bool PolygonIsOpened (CLxUser_Mesh& mesh, CLxUser_Polygon& polygon)
{
    unsigned nvert;
    polygon.VertexCount(&nvert);
    if (nvert < 3)
        return true;

    CLxUser_Edge edge;
    edge.fromMesh(mesh);

    for (auto i = 0u; i < nvert; i++)
    {
        LXtPointID A, B;
        polygon.VertexByIndex(i, &A);
        polygon.VertexByIndex((i + 1) % nvert, &B);
        edge.SelectEndpoints(A, B);
        unsigned npol;
        edge.PolygonCount(&npol);
        if (npol != 1)
            return false;
    }

    return true;
}

//
// Make position vectors on the axis plane.
//
static void MakePositionVectors(CLxUser_Mesh& mesh, AxisPlane& axisPlane, std::vector<LXtPointID>& points, std::vector<CLxVector>& positions)
{
    CLxUser_Point point;
    point.fromMesh(mesh);

    positions.clear();

    for (auto v : points)
    {
        LXtFVector pos;
        point.Select(v);
        point.Pos(pos);
        double x, y, z;
        axisPlane.ToPlane(pos, x, y, z);
        positions.push_back(CLxVector(x, y, z));
    }
}

static bool IsIntersected(CLxVector p0, CLxVector p1, CLxVector q0, CLxVector q1)
{
    if (p0 == q0 || p0 == q1 || p1 == q0 || p1 == q1)
        return false;

    CLxVector u = p1 - p0;
    CLxVector v = q1 - q0;
    CLxVector w = p0 - q0;

    int ix = 0, iy = 1;

    double D = u[ix] * v[iy] - u[iy] * v[ix];
    if (std::abs(D) < lx::Tolerance(D))
        return false;

    double s = (v[ix] * w[iy] - v[iy] * w[ix]) / D;
    if (s < 0.0 || s > 1.0)
        return false;

    double t = (u[ix] * w[iy] - u[iy] * w[ix]) / D;
    if (t < 0.0 || t > 1.0)
        return false;

    return true;
}

static bool BridgeIsIntesected(CLxVector p0, CLxVector p1, std::vector<CLxVector>& loop)
{
    for (auto i = 0u; i < loop.size(); i++)
    {
        auto j = (i + 1) % loop.size();
        if (IsIntersected(p0, p1, loop[i], loop[j]))
            return true;
    }
    return false;
}

//
// Merge holes with outer loop and make a keyhole polygon. This ignores selected edges even if they are on border.
//
static void MakeKeyhole(CLxUser_Mesh& mesh, AxisPlane& axisPlane, std::vector<LXtPointID>& outer, std::vector<std::vector<LXtPointID>>& holes, std::vector<LXtPointID>& keyhole)
{
    std::vector<CLxVector> outer_vectors;
    keyhole = outer;

    CLxUser_Point point;
    point.fromMesh(mesh);
    CLxUser_MeshService s_mesh;
    LXtMarkMode mark_used = s_mesh.ClearMode(LXsMARK_USER_1);
    for (auto v : keyhole)
    {
        point.Select(v);
        point.SetMarks(mark_used);
    }
    mark_used = s_mesh.SetMode(LXsMARK_USER_1);

    printf("MakeKeyhole outer.size() = %lu holes = %lu\n", outer.size(), holes.size());
    for (auto& hole : holes)
    {
        MakePositionVectors(mesh, axisPlane, keyhole, outer_vectors);
        printf("-- hole.size() = %lu\n", hole.size());
        std::vector<CLxVector> hole_vectors;
        MakePositionVectors(mesh, axisPlane, hole, hole_vectors);
        double min_dist = std::numeric_limits<double>::max();
        unsigned min_index = 0, hole_index = 0;
        for (auto i = 0u; i < outer_vectors.size(); i++)
        {
            point.Select(keyhole[i]);
            if (point.TestMarks(mark_used) == LXe_TRUE)
                continue;
            for (auto j = 0u; j < hole_vectors.size(); j++)
            {
                point.Select(hole[j]);
                if (point.TestMarks(mark_used) == LXe_TRUE)
                    continue;
                double dist = (outer_vectors[i] - hole_vectors[j]).length();
                if (dist < min_dist)
                {
                    if (BridgeIsIntesected(outer_vectors[i], hole_vectors[j], outer_vectors))
                        continue;
                    min_dist = dist;
                    min_index = i;
                    hole_index = j;
                }
            }
        }
        std::vector<LXtPointID> temp;
        for (auto j = 0u; j < keyhole.size(); j++)
        {
            temp.push_back(keyhole[j]);
            if (j == min_index)
            {
                for (auto k = 0u; k < hole.size(); k++)
                {
                    temp.push_back(hole[(hole_index + k) % hole.size()]);
                }
                temp.push_back(hole[hole_index]);
                temp.push_back(keyhole[j]);

                point.Select(hole[hole_index]);
                point.SetMarks(mark_used);
                point.Select(keyhole[j]);
                point.SetMarks(mark_used);
            }
        }
        keyhole = temp;
    }
    printf("** keyhole.size() = %lu\n", keyhole.size());
}

//
// Make border vertex loops in the mesh.
//
static bool FirstBorderEdge(CLxUser_Mesh& mesh, CLxUser_Edge& edge)
{
    CLxUser_MeshService mesh_svc;
    LXtMarkMode mark_done = mesh_svc.SetMode(LXsMARK_USER_0);
    LXtMarkMode mark_select = mesh_svc.SetMode(LXsMARK_SELECT);

    unsigned nedge;
    mesh.EdgeCount(&nedge);
    for (auto i = 0u; i < nedge; i++)
    {
        edge.SelectByIndex(i);
        if (edge.TestMarks(mark_done) == LXe_TRUE)
            continue;
        if (edge.TestMarks(mark_select) == LXe_TRUE)
            continue;
        if (edge.IsBorder() == LXe_TRUE)
        {
            edge.SetMarks(mark_done);
            return true;
        }
    }
    return false;
}
static bool NextBorderEdge(CLxUser_Mesh& mesh, CLxUser_Edge& edge, LXtPointID pntID, LXtPointID* nextID)
{
    CLxUser_Point point;
    point.fromMesh(mesh);
    point.Select(pntID);

    CLxUser_Edge edge1;
    edge1.fromMesh(mesh);

    CLxUser_MeshService mesh_svc;
    LXtMarkMode mark_done = mesh_svc.SetMode(LXsMARK_USER_0);
    LXtMarkMode mark_select = mesh_svc.SetMode(LXsMARK_SELECT);

    unsigned count;
    point.EdgeCount(&count);
    for (auto i = 0u; i < count; i++)
    {
        LXtEdgeID edgeID;
        point.EdgeByIndex(i, &edgeID);
        if (edgeID == edge.ID())
            continue;
        edge1.Select(edgeID);
        if (edge1.TestMarks(mark_done) == LXe_TRUE)
            continue;
        if (edge1.TestMarks(mark_select) == LXe_TRUE)
            continue;
        if (edge1.IsBorder() == LXe_TRUE)
        {
            LXtPointID pntID0, pntID1;
            edge.Select(edgeID);
            edge.SetMarks(mark_done);
            edge.Endpoints(&pntID0, &pntID1);
            *nextID = (pntID0 == pntID) ? pntID1 : pntID0;
            return true;
        }
    }
    return false;
}

static bool LoopIsReversed(CLxUser_Mesh& mesh, std::vector<LXtPointID>& loop)
{
    CLxUser_Edge edge;
    edge.fromMesh(mesh);
    CLxUser_Polygon poly;
    poly.fromMesh(mesh);

    LXtPolygonID polID;
    edge.SelectEndpoints(loop.at(0), loop.at(1));
    edge.PolygonByIndex(0, &polID);
    poly.Select(polID);
    unsigned count;
    poly.VertexCount(&count);
    for(auto i = 0; i < count; i++)
    {
        auto j = (i + 1) % count;
        LXtPointID v0, v1;
        poly.VertexByIndex(i, &v0);
        poly.VertexByIndex(j, &v1);
        if ((v0 == loop.at(0)) && (v1 == loop.at(1)))
            return true;
        else if ((v0 == loop.at(1)) && (v1 == loop.at(0)))
            return false;

    }
    return true;
}

static bool MakeBorderLoops(CLxUser_Mesh& mesh, std::vector<std::vector<LXtPointID>>& loops)
{
    CLxUser_Edge edge;
    edge.fromMesh(mesh);

    CLxUser_MeshService mesh_svc;
    LXtMarkMode mark_done;

    mark_done = mesh_svc.ClearMode(LXsMARK_USER_0);

    unsigned nedge;
    mesh.EdgeCount(&nedge);
    for (auto i = 0u; i < nedge; i++)
    {
        edge.SelectByIndex(i);
        edge.SetMarks(mark_done);
    }

    loops.clear();

    unsigned idx;
    while (FirstBorderEdge (mesh, edge))
    {
        std::vector<LXtPointID> loop;
        LXtPointID pntID0, pntID1, pntID;
        edge.Endpoints(&pntID0, &pntID1);
        loop.push_back(pntID0);
        loop.push_back(pntID1);
        edge.Index(&idx);
        //printf("FirstBorderEdge (%u)\n", idx);
        LXtPointID lastID = pntID1;
        bool closed = false;
        while (NextBorderEdge (mesh, edge, lastID, &pntID))
        {
            if (pntID == loop.front())
            {
                closed = true;
                if (LoopIsReversed(mesh, loop))
                    std::reverse(loop.begin(), loop.end());
                loops.push_back(loop);
                break;
            }
            else
            {
                loop.push_back(pntID);
                edge.SelectEndpoints(lastID, pntID);
                edge.Index(&idx);
                //printf("-- next (%u)\n", idx);
                lastID = pntID;
            }
        }
        //printf("   closed (%d) (%zu)\n", closed, loop.size());
        if (closed)
            continue;
        lastID = pntID0;
        edge.SelectEndpoints(pntID0, pntID1);
        while (NextBorderEdge (mesh, edge, lastID, &pntID))
        {
            if (pntID == loop.back())
            {
                loops.push_back(loop);
                break;
            }
            else
            {
                loop.insert(loop.begin(), pntID);
                edge.SelectEndpoints(lastID, pntID);
                lastID = pntID;
            }
        }
    }

    return true;
}

static bool IsDiscoEdge(CLxUser_Mesh& mesh, CLxUser_MeshMap& vmap, CLxUser_Edge edge)
{
    if (!vmap.ID())
        return false;

    unsigned count;
    edge.PolygonCount(&count);
    if (count == 1)
    {
        return true;
    }

    LXtPointID pntID0, pntID1;
    edge.Endpoints(&pntID0, &pntID1);
    LXtPolygonID polID0, polID1;
    edge.PolygonByIndex(0, &polID0);
    edge.PolygonByIndex(1, &polID1);

    LXtID4 type;
    vmap.Type(&type);
    if (type != LXi_VMAP_TEXTUREUV)
        return false;

    float uv0[2], uv1[2];
    CLxUser_Polygon poly;
    poly.fromMesh(mesh);

    poly.Select(polID0);
    poly.MapEvaluate(vmap.ID(), pntID0, uv0);
    poly.Select(polID1);
    poly.MapEvaluate(vmap.ID(), pntID0, uv1);
    if (MathUtil::VectorEqual(uv0, uv1, 2) == false)
    {
        return true;
    }

    poly.Select(polID0);
    poly.MapEvaluate(vmap.ID(), pntID1, uv0);
    poly.Select(polID1);
    poly.MapEvaluate(vmap.ID(), pntID1, uv1);
    if (MathUtil::VectorEqual(uv0, uv1, 2) == false)
    {
        return true;
    }

    return false;
}

static bool TriangleNormal(CLxUser_Mesh& mesh, LXtPointID v0, LXtPointID v1, LXtPointID v2, LXtVector norm)
{
    CLxUser_Point point;
    point.fromMesh(mesh);

    LXtFVector pos;
    point.Select(v0);
    point.Pos(pos);
    LXtVector pos0;
    LXx_VCPY(pos0, pos);

    point.Select(v1);
    point.Pos(pos);
    LXtVector pos1;
    LXx_VCPY(pos1, pos);

    point.Select(v2);
    point.Pos(pos);
    LXtVector pos2;
    LXx_VCPY(pos2, pos);

    return MathUtil::CrossNormal(norm, pos0, pos1, pos2);
}

// Get the orientation of the give vertex list
static bool VertexListOrientation(CLxUser_Mesh& mesh, AxisPlane& axisPlane, std::vector<LXtPointID>& points)
{
    CLxUser_Point point;
    LXtFVector pos;
    double x0, y0, x1, y1, z;

    point.fromMesh(mesh);
    point.Select(points.front());
    point.Pos(pos);
    axisPlane.ToPlane(pos, x0, y0, z);
    point.Select(points.back());
    point.Pos(pos);
    axisPlane.ToPlane(pos, x1, y1, z);
    double area = x1 * y0 - x0 * y1;

    for (auto i = 1u; i < points.size(); i++)
    {
        point.Select(points[i]);
        point.Pos(pos);
        axisPlane.ToPlane(pos, x1, y1, z);
        area += x0 * y1 - x1 * y0;
        x0 = x1;
        y0 = y1;
    }

	return (area >= 0.0);
}

// Get vertex list of the given polygon. If the first vertex is not convex, it fixes the order of vertices.
static bool PolygonFixedVertexList(CLxUser_Mesh& mesh, CLxUser_Polygon& poly, std::vector<LXtPointID>& points)
{
    LXtVector norm;
    poly.Normal(norm);

    AxisPlane axisPlane(MathUtil::MaxExtent(norm));

    unsigned nvert;
    poly.VertexCount(&nvert);
    points.resize(nvert);

    for (auto i = 0u; i < nvert; i++)
    {
        LXtPointID pntID;
        poly.VertexByIndex(i, &pntID);
        points[i] = pntID;
    }

    if (nvert < 4)
        return false;

    auto orient = VertexListOrientation(mesh, axisPlane, points);

    // The first vertex is convex
    int i = norm[axisPlane.m_axis] < 0.0;
    if ((i ^ orient))
        return false;

    CLxUser_Point point;
    point.fromMesh(mesh);

    LXtFVector prev, curr, next;
    double dmax = 0.0;
    unsigned index = 0;

    for (auto i = 0u; i < nvert; i++)
    {
        point.Select(points[(i - 1 + nvert) % nvert]);
        point.Pos(prev);
        point.Select(points[i]);
        point.Pos(curr);
        point.Select(points[(i + 1) % nvert]);
        point.Pos(next);
        double d = axisPlane.Determ(prev, curr, next);
        if (((d >= 0.0) == orient) && (std::abs(d) >= dmax))
        {
            dmax = std::abs(d);
            index = i;
        }
    }

#if 0
    int poly_index;
    poly.Index(&poly_index);
    CLxUser_StringTag polyTag;
    polyTag.set(poly);
    const char* matr = polyTag.Value(LXi_PTAG_MATR);
#endif
    if (index == 0)
        return false;
    
    std::vector<LXtPointID> temp;
    temp = points;
    for (auto i = 0u; i < points.size(); i++)
    {
        points[i] = temp[(i + index) % points.size()];
    }
    return true;
}

static double TriangleDeterm (CLxUser_Mesh& mesh, AxisPlane& axisPlane, LXtPointID v1, LXtPointID v2, LXtPointID v3)
{
    CLxUser_Point point;
    point.fromMesh(mesh);

    LXtFVector pos1, pos2, pos3;

    point.Select(v1);
    point.Pos(pos1);
    point.Select(v2);
    point.Pos(pos2);
    point.Select(v3);
    point.Pos(pos3);

    return axisPlane.Determ(pos1, pos2, pos3);
}

static bool TriangleCorner (CLxUser_Mesh& mesh, AxisPlane& axisPlane, bool orient, LXtPointID v1, LXtPointID v2, LXtPointID v3)
{
    double d = TriangleDeterm(mesh, axisPlane, v1, v2, v3);
    if (!d)
        return false;
	return (d > 0.0) == orient;
}

// Check the orientation of the triangle on the axis plane in the given vertex list.
static bool TriangleOriented (CLxUser_Mesh& mesh, AxisPlane& axisPlane, std::vector<LXtPointID>& points, bool orient, LXtPointID v1, LXtPointID v2, LXtPointID v3)
{
	// Check the determination of the triangle.
    double d = TriangleDeterm(mesh, axisPlane, v1, v2, v3);
	if ((d >= 0.0) != orient)
		return false;

	for (auto i = 0u; i < points.size(); i++) {
		LXtPointID v = points[i];
		if ((v == v1) || (v == v2) || (v == v3))
			continue;
		if (!TriangleCorner (mesh, axisPlane, orient, v2, v1, v) &&
            !TriangleCorner (mesh, axisPlane, orient, v1, v3, v) &&
            !TriangleCorner (mesh, axisPlane, orient, v3, v2, v))
			return false;
	}
	return true;
}

}; // MeshUtil

