//
// Parameterization helper class
//
#pragma once

#include <lxsdk/lx_mesh.hpp>
#include <lxsdk/lx_select.hpp>
#include <lxsdk/lx_seltypes.hpp>
#include <lxsdk/lx_value.hpp>
#include <lxsdk/lxu_math.hpp>
#include <lxsdk/lx_log.hpp>

#include <Eigen/Dense>

#include <memory>
#include <vector>
#include <string>

struct CParam
{
    enum : unsigned
    {
        DISCO_SEAM    = 0x01,
        DISCO_LOCKED  = 0x02,
    };

#if 0
    enum : unsigned
    {
        FLAG_SEAL     = 0x01,
        FLAG_LAYOUT   = 0x02,
        FLAG_USEMATR  = 0x0100,
        FLAG_USEPART  = 0x0200,
        FLAG_LOADLOCK = 0x80000
    };
#endif

    enum : unsigned
    {
        METHOD_LSCM      = 0,
        METHOD_ARAP      = 1,
        METHOD_BORDER    = 2,
        METHOD_SLIM      = 3
    };

    enum : unsigned
    {
        BORDER_CIRCULAR= 0,
        BORDER_SQUARE = 1,
    };

    enum : unsigned
    {
        PARAM_DISCRETE_CONFORMAL = 0,   // Angle
        PARAM_DISCRETE_AUTHANLIC = 1,   // Area
        PARAM_MVC = 2,  // Smooth
        PARAM_BARYCENTRIC = 3,  // Fast
    };

    struct ParamVerx;
    struct ParamEdge;
    struct ParamTriangle;
    struct ParamGroup;

    typedef std::shared_ptr<ParamVerx>     ParamVerxID;
    typedef std::shared_ptr<ParamEdge>     ParamEdgeID;
    typedef std::shared_ptr<ParamTriangle> ParamTriangleID;
    typedef std::shared_ptr<ParamGroup>    ParamGroupID;

    struct ParamVerx
    {
        ParamTriangleID               tri;
        LXtPointID                    vrt;
        unsigned                      vrt_index;
        float                         value[2];   // texture coordinate value
 //       float                         value0[2];  // texture initial value
 //       float                         value1[2];  // value for working
        unsigned                      index;      // solver index
        unsigned                      flags;      // LSCM flags
        LXtMarkMode                   marks;      // marks for working
        float                         w;          // falloff weight
        LXtFVector                    pos;        // vertex corner position
        void*                         userData;   // any working data
        ParamEdgeID                   edge;       // an edge */
        std::vector<ParamTriangleID>  tris;       // connecting triangles
    };

    struct ParamEdge
    {
        ParamTriangleID tri;     // source triangle
        ParamVerxID     v0, v1;  // vertex 1,2
        ParamEdgeID     next;    // next edge
        ParamEdgeID     pair;    // opposite edge
        int             index;   // solver index
    };

    struct ParamTriangle
    {
        LXtPolygonID pol;
        int          pol_index;
        ParamVerxID  v0, v1, v2;
        unsigned     group;  // group index
        double       area;
        void*        userData;  // any working data
        int          index;     // solver index
        ParamEdgeID  edge;      // an edge
    };

    struct ParamFace
    {
        unsigned                     group;  // group index
        std::vector<ParamTriangleID> tris;   // triangles of the face
    };

    struct ParamGroup
    {
        unsigned       index;
        CLxBoundingBox box2D, box3D;
        double         size;
        double         scale; /* area 3D / area 2D */
        int            nx, ny;
        int            k0;
        double         x0, y0;
        double         wx, wy;
        double         mag;
        double         area2D, area3D;
        bool           locked;
        unsigned       flipped; // number of flipped triangles

        int    ix, iy;
        double ox, oy; /* UDIM position */
    
        ParamVerxID     vxmax, vxmin;   // max and min vertex

        std::vector<ParamTriangleID> tris;    // connecting triangles
        std::vector<ParamVerxID>     discos;  // dico vertex of the triangles
    };

    std::vector<ParamGroupID>    m_groups;
    std::vector<ParamEdgeID>     m_edges;
    std::vector<ParamVerxID>     m_vertices;
    std::vector<ParamTriangleID> m_triangles;

    std::unordered_map<LXtPolygonID, ParamFace> m_faces;

    CLxUser_Mesh    m_mesh;
    CLxUser_MeshMap m_vmap;
    CLxUser_Polygon m_poly;
    CLxUser_Point   m_vert;
    unsigned int    m_flags;
    double          m_region[4];
    double          m_size_w, m_size_h;
    double          m_scale;
    double          m_gaps;
    LXtMeshMapID    m_vmap_id;
    bool            m_layout;
//    bool            m_keepAspect;
    bool            m_relax;
    LXtID4          m_type;
    std::string     m_name;

    // initial projection
//    unsigned        m_proj;
//    unsigned        m_axis;
    unsigned        m_pinn;
//    double          m_angle;

    LXtMarkMode m_pick;
    LXtMarkMode m_mark_done;
    LXtMarkMode m_mark_seam;
    LXtMarkMode m_mark_hide;
    LXtMarkMode m_mark_lock;

    CLxUser_LogService   s_log;

    CParam(CLxUser_Mesh& mesh, std::string& name)
    {
        m_name = name;
        m_mesh.set(mesh);
 //       m_flags     = 0u;
        m_region[0] = m_region[1] = 0.0;
        m_region[2] = m_region[3] = 1.0;
        m_size_w = m_size_h = 1.0;
        m_scale  = 1.0;
        m_gaps   = 0.2;
        m_layout = false;
 //       m_keepAspect = true;
        m_type  = LXiSEL_EDGE;
        m_relax = false;

    //    m_proj = PROJ_USEVMAP;
    //    m_axis = 1;
        m_pinn = 0;

        m_poly.fromMesh(m_mesh);
        m_vert.fromMesh(m_mesh);
        m_vmap.fromMesh(m_mesh);
        m_vmap.SelectByName(LXi_VMAP_TEXTUREUV, m_name.c_str());

        CLxUser_MeshService mesh_svc;
        m_pick      = mesh_svc.SetMode(LXsMARK_SELECT);
        m_mark_done = mesh_svc.SetMode(LXsMARK_USER_0);
        m_mark_seam = mesh_svc.SetMode(LXsMARK_USER_1);
        m_mark_hide = mesh_svc.SetMode(LXsMARK_HIDE);
        m_mark_lock = mesh_svc.SetMode(LXsMARK_LOCK);
    }

    void Clear()
    {
        m_vertices.clear();
        m_edges.clear();
        m_triangles.clear();
        m_faces.clear();
        m_groups.clear();
    }

    LxResult Setup(LXtMarkMode edge_mark, bool seal, bool relax);
    LxResult SealHoles();

    LxResult LSCM(int pinn);
    LxResult ARAP();
    LxResult Border(unsigned border, unsigned param);
    LxResult SLIM(int iter);

    // V : 3d vertex positions
    // F : vertex indices of triangles
    // V_o : initial 2d uv positions
    // b : pinned vertex indices
    // bc : 2d positions of pinnded vertices.
    LxResult EgenMatrix(ParamGroupID grp, Eigen::MatrixXd& V, Eigen::MatrixXi& F, Eigen::MatrixXd& V_o, Eigen::VectorXi& b, Eigen::MatrixXd& bc);
 //   LxResult SetResult(ParamGroupID grp, Eigen::MatrixXd& V_o);

    LxResult Apply(CLxUser_Mesh& edit_mesh, double gaps);


//    LxResult         Project();
    LxResult         AddTriangle(LXtPolygonID pol, LXtPointID v0, LXtPointID v1, LXtPointID v2);
    LxResult         AddPolygon(LXtPolygonID pol);
    LXtPolygonID     TracePolygon(LXtPointID vrt, LXtPolygonID pol, int shift);
    ParamTriangleID  FetchTriangle(LXtPointID vrt, LXtPolygonID pol);
    ParamVerxID      FetchVertex(LXtPointID vrt);
    ParamVerxID      AddVertex(LXtPointID vrt, LXtPolygonID pol, ParamTriangleID tri);
    bool             IsSeamEdge(CLxUser_Edge& edge);
//    void             ProjectionAxis(CLxBoundingBox& box, LXtVector p1, LXtVector p2);
    bool             LockVertex(ParamVerxID dv);
//    unsigned int     ProjectPlanar(ParamGroupID grp, LXtMatrix& m);
//    unsigned int     ProjectCylinder(ParamGroupID grp, LXtMatrix& m);
//   unsigned int     ProjectSpherical(ParamGroupID grp, LXtMatrix& m);
    double           Layout(double gaps);
    bool             SetUVs(ParamTriangleID tri, double mag);
    bool             FitGroup(ParamGroupID grp, unsigned N, std::vector<bool>& grid);
    bool             ConnectPols(ParamVerxID dv, LXtPolygonID pol, const float* value);
    void             SetGroupInfo(ParamGroupID grp);
//    void             ScaleGroup(ParamGroupID grp);
    void             GetPins(ParamGroupID grp, int pinn, ParamVerxID* pin1, ParamVerxID* pin2);
    void             LoadUVs(ParamGroupID grp);
};
