//
// Parametarize a mesh for UV unwrapping.
//

#include "tool.hpp"
#include "command.hpp"

/*
 * On create we add our one tool attribute. We also allocate a vector type
 * and select mode mask.
 */
CTool::CTool()
{
    static LXtTextValueHint method_hint[] = {
        { CParam::METHOD_LSCM, "lscm" }, 
        { CParam::METHOD_ARAP, "arap" }, 
        { CParam::METHOD_BORDER, "border" }, 
        { CParam::METHOD_SLIM, "slim" }, 
        { 0, "=parameterize_method" }, 
        { 0, nullptr },
    };

    static LXtTextValueHint border_hint[] = {
        { CParam::BORDER_CIRCULAR, "circular" },
        { CParam::BORDER_SQUARE, "square" },
        { 0, "=parameterize_border" },
        { 0, nullptr },
    };

    static LXtTextValueHint param_hint[] = {
        { CParam::PARAM_DISCRETE_CONFORMAL, "angle" },
        { CParam::PARAM_DISCRETE_AUTHANLIC, "area" },
        { CParam::PARAM_MVC, "smooth" },
        { CParam::PARAM_BARYCENTRIC, "fast" },
        { 0, "=parameterize_border" },
        { 0, nullptr },
    };

    CLxUser_PacketService sPkt;
    CLxUser_MeshService   sMesh;

    dyna_Add(ATTRs_NAME, LXsTYPE_VERTMAPNAME);

    dyna_Add(ATTRs_METHOD, LXsTYPE_INTEGER);
    dyna_SetHint(ATTRa_METHOD, method_hint);

    dyna_Add(ATTRs_BORDER, LXsTYPE_INTEGER);
    dyna_SetHint(ATTRa_BORDER, border_hint);

    dyna_Add(ATTRs_PARAM, LXsTYPE_INTEGER);
    dyna_SetHint(ATTRa_PARAM, param_hint);

    dyna_Add(ATTRs_SEAL, LXsTYPE_BOOLEAN);

    dyna_Add(ATTRs_ITERATION, LXsTYPE_INTEGER);

    dyna_Add(ATTRs_RELAX, LXsTYPE_BOOLEAN);

    dyna_Add(ATTRs_GAPS, LXsTYPE_PERCENT);

    dyna_Add(ATTRs_PINN, LXsTYPE_AXIS);

    tool_Reset();

    sPkt.NewVectorType(LXsCATEGORY_TOOL, v_type);
    sPkt.AddPacket(v_type, LXsP_TOOL_VIEW_EVENT, LXfVT_GET);
    sPkt.AddPacket(v_type, LXsP_TOOL_SCREEN_EVENT, LXfVT_GET);
    sPkt.AddPacket(v_type, LXsP_TOOL_FALLOFF, LXfVT_GET);
    sPkt.AddPacket(v_type, LXsP_TOOL_SUBJECT2, LXfVT_GET);
    sPkt.AddPacket(v_type, LXsP_TOOL_INPUT_EVENT, LXfVT_GET);
	sPkt.AddPacket (v_type, LXsP_TOOL_EVENTTRANS,  LXfVT_GET);
	sPkt.AddPacket (v_type, LXsP_TOOL_ACTCENTER,   LXfVT_GET);

    offset_view = sPkt.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_VIEW_EVENT);
    offset_screen = sPkt.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_SCREEN_EVENT);
    offset_falloff = sPkt.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_FALLOFF);
    offset_subject = sPkt.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_SUBJECT2);
    offset_input = sPkt.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_INPUT_EVENT);
	offset_event  = sPkt.GetOffset (LXsCATEGORY_TOOL, LXsP_TOOL_EVENTTRANS);
	offset_center = sPkt.GetOffset (LXsCATEGORY_TOOL, LXsP_TOOL_ACTCENTER);
    mode_select = sMesh.SetMode("select");

    m_part    = -1;
    m_offset0 = 0.0;
}

/*
 * Reset sets the attributes back to defaults.
 */
void CTool::tool_Reset()
{
    dyna_Value(ATTRa_NAME).SetString("Texture");
    dyna_Value(ATTRa_METHOD).SetInt(CParam::METHOD_LSCM);
    dyna_Value(ATTRa_BORDER).SetInt(CParam::BORDER_CIRCULAR);
    dyna_Value(ATTRa_PARAM).SetInt(CParam::PARAM_DISCRETE_CONFORMAL);
    dyna_Value(ATTRa_SEAL).SetInt(1);
    dyna_Value(ATTRa_ITERATION).SetInt(0);
    dyna_Value(ATTRa_RELAX).SetInt(0);
    dyna_Value(ATTRa_GAPS).SetFlt(0.2);
    dyna_Value(ATTRa_PINN).SetInt(0);
}

/*
 * Boilerplate methods that identify this as an action (state altering) tool.
 */
LXtObjectID CTool::tool_VectorType()
{
    return v_type.m_loc;  // peek method; does not add-ref
}

const char* CTool::tool_Order()
{
    return LXs_ORD_ACTR;
}

LXtID4 CTool::tool_Task()
{
    return LXi_TASK_ACTR;
}

LxResult CTool::tool_GetOp(void** ppvObj, unsigned flags)
{
    CLxSpawner<CToolOp> spawner(SRVNAME_TOOLOP);
    CToolOp*            toolop = spawner.Alloc(ppvObj);

	if (!toolop)
	{
		return LXe_FAILED;
	}

    dyna_Value(ATTRa_METHOD).GetInt(&toolop->m_method);
    dyna_Value(ATTRa_NAME).GetString(toolop->m_name);
    dyna_Value(ATTRa_BORDER).GetInt(&toolop->m_border);
    dyna_Value(ATTRa_PARAM).GetInt(&toolop->m_param);
    dyna_Value(ATTRa_SEAL).GetInt(&toolop->m_seal);
    dyna_Value(ATTRa_ITERATION).GetInt(&toolop->m_iter);
    dyna_Value(ATTRa_RELAX).GetInt(&toolop->m_relax);
    dyna_Value(ATTRa_GAPS).GetFlt(&toolop->m_gaps);
    dyna_Value(ATTRa_PINN).GetInt(&toolop->m_pinn);

    if (toolop->m_name.length() == 0)
        toolop->m_name = "Texture";

    toolop->offset_view = offset_view;
    toolop->offset_screen = offset_screen;
    toolop->offset_falloff = offset_falloff;
    toolop->offset_subject = offset_subject;
    toolop->offset_input = offset_input;
    toolop->m_flags = flags;

	return LXe_OK;
}

LXtTagInfoDesc CTool::descInfo[] =
{
	{LXsTOOL_PMODEL, "."},
	{LXsTOOL_USETOOLOP, "."},
	{LXsPMODEL_SELECTIONTYPES, LXsSELOP_TYPE_POLYGON "," LXsSELOP_TYPE_EDGE},
	{0}

};

/*
 * We employ the simplest possible tool model -- default hauling. We indicate
 * that we want to haul one attribute, we name the attribute, and we implement
 * Initialize() which is what to do when the tool activates or re-activates.
 * In this case set the axis to the current value.
 */
unsigned CTool::tmod_Flags()
{
    return LXfTMOD_I0_INPUT | LXfTMOD_DRAW_3D;
}

LxResult CTool::tmod_Enable(ILxUnknownID obj)
{
    CLxUser_Message msg(obj);

    if (TestPolygon() == false)
    {
        msg.SetCode(LXe_CMD_DISABLED);
        msg.SetMessage(SRVNAME_TOOL, "NoPolygon", 0);
        return LXe_DISABLED;
    }
    return LXe_OK;
}

void CTool::tmod_Initialize(ILxUnknownID vts, ILxUnknownID adjust, unsigned int flags)
{
	if (!(flags & LXfINITIALIZE_PROCEDURAL))
    {
        CLxUser_SelectionService s_sel;
        CLxUser_VMapPacketTranslation	 pkt_vmap;
        int count = s_sel.Count(LXi_VMAP_TEXTUREUV);
        if (count)
        {
            void* pkt = s_sel.Recent(LXi_VMAP_TEXTUREUV);
            if (pkt)
            {
                const char* name;
                pkt_vmap.autoInit();
                pkt_vmap.Name(pkt, &name);
                dyna_Value(ATTRa_NAME).SetString(name);
            }
        }
    }
}

void CTool::atrui_UIHints2(unsigned int index, CLxUser_UIHints& hints)
{
    switch (index)
    {
        case ATTRa_NAME:
            hints.VertmapType (LXi_VMAP_TEXTUREUV);
            hints.VertmapAllowNone (1);
            break;
        
        case ATTRa_ITERATION:
            hints.MinInt(0);
            break;

	    case ATTRa_GAPS:
            hints.MinFloat (0);
            hints.MaxFloat (0.5);
            break;
    }
}

bool CTool::TestPolygon()
{
    /*
     * Start the scan in read-only mode.
     */
    CLxUser_LayerScan scan;
    CLxUser_Mesh      mesh;
    unsigned          i, n, count;
    bool              ok = false;

    s_layer.BeginScan(LXf_LAYERSCAN_ACTIVE | LXf_LAYERSCAN_MARKPOLYS, scan);

    /*
     * Count the polygons in all mesh layers.
     */
    if (scan)
    {
        n = scan.NumLayers();
        for (i = 0; i < n; i++)
        {
            scan.BaseMeshByIndex(i, mesh);
            mesh.PolygonCount(&count);
            if (count > 0)
            {
                ok = true;
                break;
            }
        }
        scan.Apply();
    }

    /*
     * Return false if there is no polygons in any active layers.
     */
    return ok;
}

/*
 * Tool evaluation uses layer scan interface to walk through all the active
 * meshes and visit all the selected polygons.
 */
LxResult CToolOp::top_Evaluate(ILxUnknownID vts)
{
    CLxUser_VectorStack vec(vts);

    /*
     * Start the scan in edit mode.
     */
    CLxUser_LayerScan  scan;
    CLxUser_Mesh       base_mesh, edit_mesh;

    if (vec.ReadObject(offset_subject, subject) == false)
        return LXe_FAILED;
    if (vec.ReadObject(offset_falloff, falloff) == false)
        return LXe_FAILED;

    CLxUser_MeshService   s_mesh;
    CLxUser_SelectionService s_sel;

    LXtID4 edge_type = s_sel.LookupType(LXsSELTYP_EDGE);

    LXtMarkMode pick = 0;

    if (subject.Type() == edge_type)
    {
        subject.BeginScan(LXf_LAYERSCAN_EDIT_EDGES, scan);
        pick = s_mesh.SetMode(LXsMARK_SELECT);
    }
    else
        subject.BeginScan(LXf_LAYERSCAN_EDIT_POLVRT, scan);

    auto n = scan.NumLayers();
    for (auto i = 0u; i < n; i++)
    {
        scan.BaseMeshByIndex(i, base_mesh);
        scan.EditMeshByIndex(i, edit_mesh);

        CParam param(base_mesh, m_name);

        // Setup the mesh for parameterization
        if (param.Setup(pick, m_seal, m_relax) != LXe_OK)
            continue;

        LxResult result = LXe_OK;

        // Export triangle mesh for debugging
        if (1)
        {
            param.MakeParamMesh(edit_mesh);
            scan.SetMeshChange(i, LXf_MESHEDIT_GEOMETRY);
            continue;
        }

        // Parameterize using LSCM method
        if (m_method == CParam::METHOD_LSCM)
            result = param.LSCM(m_pinn);
        else if (m_method == CParam::METHOD_ARAP)
            result = param.ARAP();
        else if (m_method == CParam::METHOD_BORDER)
            result = param.Border(static_cast<unsigned>(m_border), static_cast<unsigned>(m_param));
        else if (m_method == CParam::METHOD_SLIM)
            result = param.SLIM(m_iter);

        if (result != LXe_OK)
            continue;

        if (param.Apply(edit_mesh, m_gaps) != LXe_OK)
            continue;

        scan.SetMeshChange(i, LXf_MESHEDIT_MAP_UV|LXf_MESHEDIT_GEOMETRY);
    }

    scan.Apply();

    if (m_flags &LXiTOOLOP_TOOLPIPE)
    {
        CLxUser_VMapPacketTranslation pkt_vmap;

        pkt_vmap.autoInit();
        void* packet = pkt_vmap.Packet(LXi_VMAP_TEXTUREUV, m_name.c_str());
        printf("** packet (%p) (%s)\n", packet, m_name.c_str());
	    LXtID4 selID = s_sel.LookupType (LXsSELTYP_VERTEXMAP);
        s_sel.Drop(selID);
        s_sel.Select(selID, packet);
    }
    return LXe_OK;
}

LxResult CToolOp::eltgrp_GroupCount(unsigned int* count)
{
    count[0] = 4;
    return LXe_OK;
}

LxResult CToolOp::eltgrp_GroupName(unsigned int index, const char** name)
{
    const static char* names[] = {"newPoly",
                            "newEdge",
                            "newVerx"};
    name[0] = names[index];
    return LXe_OK;
}

LxResult CToolOp::eltgrp_GroupUserName(unsigned int index, const char** username)
{
    const static char* names[] = {"@tool.offset@newPoly@0",
                            "@tool.offset@newEdge@",
                            "@tool.offset@newVerx@"};
    username[0] = names[index];
    return LXe_OK;
}

LxResult CToolOp::eltgrp_TestPolygon(unsigned int index, LXtPolygonID polygon)
{
    if (index == 0)
    {
        if (m_polygon_set.find(polygon) != m_polygon_set.end())
            return LXe_TRUE;
    }
    return LXe_FALSE;
}

LxResult CToolOp::eltgrp_TestEdge(unsigned int index, LXtEdgeID edge)
{
	LXtPointID   p0, p1;

    m_cedge.Select(edge);
    m_cedge.Endpoints(&p0, &p1);
    if (m_point_set.find(p0) == m_point_set.end())
        return LXe_FALSE;
    if (m_point_set.find(p1) == m_point_set.end())
        return LXe_FALSE;
    return LXe_TRUE;
}

LxResult CToolOp::eltgrp_TestPoint(unsigned int index, LXtPointID point)
{
    if (m_point_set.find(point) != m_point_set.end())
        return LXe_TRUE;
    return LXe_FALSE;
}

/*
 * Export tool server.
 */
void initialize()
{
    CLxGenericPolymorph* srv;

    srv = new CLxPolymorph<CTool>;
    srv->AddInterface(new CLxIfc_Tool<CTool>);
    srv->AddInterface(new CLxIfc_ToolModel<CTool>);
    srv->AddInterface(new CLxIfc_Attributes<CTool>);
    srv->AddInterface(new CLxIfc_AttributesUI<CTool>);
    srv->AddInterface(new CLxIfc_ChannelUI<CTool>);
    srv->AddInterface(new CLxIfc_StaticDesc<CTool>);
    thisModule.AddServer(SRVNAME_TOOL, srv);

    srv = new CLxPolymorph<CToolOp>;
    srv->AddInterface(new CLxIfc_ToolOperation<CToolOp>);
    srv->AddInterface(new CLxIfc_MeshElementGroup<CToolOp>);
    lx::AddSpawner(SRVNAME_TOOLOP, srv);

    CCommand::initialize();
}
