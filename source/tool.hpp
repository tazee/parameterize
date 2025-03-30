//
// Parametarize a mesh for UV unwrapping.
//

#pragma once

#include <lxsdk/lxu_attributes.hpp>
#include <lxsdk/lxu_select.hpp>
#include <lxsdk/lxu_attributes.hpp>

#include <lxsdk/lx_layer.hpp>
#include <lxsdk/lx_mesh.hpp>
#include <lxsdk/lx_log.hpp>
#include <lxsdk/lx_plugin.hpp>
#include <lxsdk/lx_seltypes.hpp>
#include <lxsdk/lx_tool.hpp>
#include <lxsdk/lx_toolui.hpp>
#include <lxsdk/lx_layer.hpp>
#include <lxsdk/lx_vector.hpp>
#include <lxsdk/lx_pmodel.hpp>
#include <lxsdk/lx_vmodel.hpp>
#include <lxsdk/lx_channelui.hpp>
#include <lxsdk/lx_draw.hpp>
#include <lxsdk/lx_handles.hpp>

#include <lxsdk/lx_value.hpp>
#include <lxsdk/lx_select.hpp>
#include <lxsdk/lx_seltypes.hpp>

#include <string>
#include <unordered_map>
#include <unordered_set>

#include "parameterize.hpp"

using namespace lx_err;

#define SRVNAME_TOOL   "tool.parameterize"
#define SRVNAME_TOOLOP "toolop.parameterize"

#define ATTRs_NAME     "name"
#define ATTRs_METHOD   "method"
#define ATTRs_BORDER   "border"
#define ATTRs_PARAM    "param"
#define ATTRs_SEAL     "seal"
#define ATTRs_ITERATION "iteration"
#define ATTRs_RELAX     "relax"
#define ATTRs_GAPS      "gaps"
#define ATTRs_PINN      "pinn"

#define ATTRa_NAME      0
#define ATTRa_METHOD    1
#define ATTRa_BORDER    2
#define ATTRa_PARAM     3
#define ATTRa_SEAL      4
#define ATTRa_ITERATION 5
#define ATTRa_RELAX     6
#define ATTRa_GAPS      7
#define ATTRa_PINN      8

#ifndef LXx_OVERRIDE
#define LXx_OVERRIDE override
#endif

//
// The Tool Operation is evaluated by the procedural modeling system.
//
class CToolOp : public CLxImpl_ToolOperation
{
	public:
        // ToolOperation Interface
		LxResult    top_Evaluate(ILxUnknownID vts)  LXx_OVERRIDE;

        CLxUser_FalloffPacket falloff;
        CLxUser_Subject2Packet subject;

        unsigned offset_view;
        unsigned offset_screen;
        unsigned offset_falloff;
        unsigned offset_subject;
        unsigned offset_input;

        unsigned m_flags;

        std::string m_name;
        int         m_method;
        int         m_border;
        int         m_param;
        int         m_seal;
        int         m_iter;
        int         m_relax;
        double      m_gaps;
        int         m_pinn;
    
        CLxUser_Edge m_cedge;
        std::unordered_set<LXtPointID> m_point_set;
        std::unordered_set<LXtPolygonID> m_polygon_set;
};

/*
 * Parameterize tool operator. Basic tool and tool model methods are defined here. The
 * attributes interface is inherited from the utility class.
 */

class CTool : public CLxImpl_Tool, public CLxImpl_ToolModel, public CLxDynamicAttributes, public CLxImpl_ChannelUI
{
public:
    CTool();

    void        tool_Reset() LXx_OVERRIDE;
    LXtObjectID tool_VectorType() LXx_OVERRIDE;
    const char* tool_Order() LXx_OVERRIDE;
    LXtID4      tool_Task() LXx_OVERRIDE;
	LxResult	tool_GetOp(void **ppvObj, unsigned flags) LXx_OVERRIDE;

    unsigned    tmod_Flags() LXx_OVERRIDE;
    LxResult    tmod_Enable(ILxUnknownID obj) LXx_OVERRIDE;
	void		tmod_Initialize (ILxUnknownID vts, ILxUnknownID adjust, unsigned flags) LXx_OVERRIDE;
    void        atrui_UIHints2(unsigned int index, CLxUser_UIHints& hints) LXx_OVERRIDE;
    LxResult	atrui_DisableMsg (unsigned int index, ILxUnknownID msg) LXx_OVERRIDE;

    LxResult    cui_Enabled           (const char *channelName, ILxUnknownID msg, ILxUnknownID item, ILxUnknownID read)	LXx_OVERRIDE;
    LxResult    cui_DependencyCount   (const char *channelName, unsigned *count) LXx_OVERRIDE;
    LxResult    cui_DependencyByIndex (const char *channelName, unsigned index, LXtItemType *depItemType, const char **depChannelName) LXx_OVERRIDE;

    using CLxDynamicAttributes::atrui_UIHints;  // to distinguish from the overloaded version in CLxImpl_AttributesUI

    bool TestPolygon();

    CLxUser_LogService   s_log;
    CLxUser_LayerService s_layer;
    CLxUser_VectorType   v_type;
    CLxUser_SelectionService s_sel;

    unsigned offset_view;
    unsigned offset_screen;
    unsigned offset_falloff;
    unsigned offset_subject;
    unsigned offset_input;
    unsigned offset_event;
	unsigned offset_center;
    unsigned mode_select;
	
	LXtItemType m_itemType;

    static LXtTagInfoDesc descInfo[];
};

