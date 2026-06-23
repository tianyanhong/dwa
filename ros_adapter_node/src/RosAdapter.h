#ifndef __ROS_ADAPTER_H__
#define __ROS_ADAPTER_H__

#include <iostream>
#include <vector>
#include <map>
#include <string.h>
#include <stdint.h>
#include <utility>

// Camel发布通行点信息
#ifndef camel_pub_topic_agv_goal
#define camel_pub_topic_agv_goal "agv_goal"
#endif

// Camel发布避障区域信息（限制区域）
#ifndef camel_pub_topic_prohibited_area
#define camel_pub_topic_prohibited_area "prohibited_area"
#endif

// Camel发布避障区域信息（允许区域）
#ifndef camel_pub_topic_accessible_area
#define camel_pub_topic_accessible_area "accessible_area"
#endif

// Camel发布匹配点信息
#ifndef camel_pub_topic_initialpose
#define camel_pub_topic_initialpose "initialpose"
#endif

// camel订阅定位结果信息
#ifndef camel_sub_topic_localization_result
#define camel_sub_topic_localization_result "/LocalizationResult"
// #define camel_sub_topic_localization_result "RealLR"
#endif

// camel订阅PLC周期发给Camel的信息
#ifndef camel_sub_topic_plc2camel_notify
#define camel_sub_topic_plc2camel_notify "plc_2_camel_notify"
#endif

// camel订阅PLC动作响应信息
#ifndef camel_sub_topic_plc2camel_action_rsp
#define camel_sub_topic_plc2camel_action_rsp "plc_2_camel_action_rsp"
#endif

// camel订阅PLC周期性数据
#ifndef camel_sub_topic_plc2camel_direct
#define camel_sub_topic_plc2camel_direct "plc_to_camel_direct"
#endif


// camel周期给ros plc的基础数据
#ifndef camel_pub_topic_camel2plc_notify
#define camel_pub_topic_camel2plc_notify "camel_2_plc_notify"
#endif

// camel发布给PLC的动作请求信息
#ifndef camel_pub_topic_camel2plc_action_req
#define camel_pub_topic_camel2plc_action_req "camel_2_plc_action_req"
#endif

// camel周期给ros plc的基础数据(直发)
#ifndef camel_pub_topic_camel2plc_notify_direct
#define camel_pub_topic_camel2plc_notify_direct "camel_to_plc_direct"
#endif

// camel发布给PLC的动作请求信息
#ifndef camel_pub_topic_odom
#define camel_pub_topic_odom "v_odom"
#endif

// camel发布给controller的base信息
#ifndef camel_pub_topic_toC_base
#define camel_pub_topic_toC_base "camel2c_base"
#endif

// camel发布给controller的points信息
#ifndef camel_pub_topic_toC_points
#define camel_pub_topic_toC_points "camel2c_points"
#endif

// camel发布给controller的末端定位信息
#ifndef camel_pub_topic_toC_terminal
#define camel_pub_topic_toC_terminal "camel2c_terminal"
#endif

// camel发布给controller的末端定位配置信息
#ifndef camel_pub_topic_toC_terminal_config
#define camel_pub_topic_toC_terminal_config "camel2c_terminal_config"
#endif

// camel订阅controller的base信息
#ifndef camel_sub_topic_fromC
#define camel_sub_topic_fromC "BsplinesController"
#endif

// camel发布给立体防护的货物尺寸信息
#ifndef camel_pub_topic_to3D_cargo_attribute
#define camel_pub_topic_to3D_cargo_attribute "/activeSafe/cargo_msg"
#endif

// camel发布给立体防护的开启关闭信息
#ifndef camel_pub_topic_to3D_is_enable
#define camel_pub_topic_to3D_is_enable "/activeSafe/is_enable"
#endif

// camel订阅的二维码相机数据用于检查是否收到数据
#ifndef camel_sub_topic_Check_QR_Camera
#define camel_sub_topic_Check_QR_Camera "/check_qr_useful"
#endif

// camel订阅的已处理的二维码相机数据
#ifndef camel_sub_topic_QR_Camera
//#define camel_sub_topic_QR_Camera "MV_IM5005_02MWG"
#define camel_sub_topic_QR_Camera "/qr_offset"
#endif

// camel订阅的告警信息
#ifndef camel_sub_topic_alarm
#define camel_sub_topic_alarm "/ErrCodeCollector"
#endif

// camel与PLC之间消息的最大长度
#ifndef camel_plc_msg_max_length
#define camel_plc_msg_max_length 1024
#endif

// Camel的末端定位服务（客户端）
#ifndef camel_ros_service_terminal_position
#define camel_ros_service_terminal_position "terminal_position"
#endif

// 导航向camel申请/释放自主避障区域的服务（服务端）
#ifndef camel_ros_service_avoid_operation
#define camel_ros_service_avoid_operation "AccessibleAreaOperation"
#endif

// 获取车体状态信息
#ifndef camel_ros_service_get_car_state
#define camel_ros_service_get_car_state "camel_car_state"
#endif

// Camel发给Ros Plc的手柄控制数据
#ifndef camel_pub_topic_camel2plc_stick
#define camel_pub_topic_camel2plc_stick "joy"
#endif

// Camel发给Season的导航切换动作
#ifndef camel_pub_topic_action_list
#define camel_pub_topic_action_list "ActionList"
#endif

// Season发给Camel的导航切换结果
#ifndef camel_sub_topic_action_feedback
#define camel_sub_topic_action_feedback "ActionFeedback"
#endif

// camel订阅marker定位结果信息
#ifndef camel_sub_topic_marker_LR
#define camel_sub_topic_marker_LR "/MarkLR"
#endif

// camel发布小车的告警信息
#ifndef camel_pub_topic_AGV_alarm
#define camel_pub_topic_AGV_alarm "/AgvAlarm"
#endif

// camel订阅控制发布的路线终点二维码偏差
#ifndef camel_sub_topic_QR_error_arrived
#define camel_sub_topic_QR_error_arrived "QRerrorArrived"
#endif

// camel发布小车的状态信息
#ifndef camel_pub_topic_AGV_Status
#define camel_pub_topic_AGV_Status "/AgvStatus"
#endif

// camel发布AGV详细状态信息
#ifndef camel_pub_topic_agv_status_info
#define camel_pub_topic_agv_status_info "/agv_status_info"
#endif

// camel发布小车按键事件
#ifndef camel_pub_topic_Car_Button_Event
#define camel_pub_topic_Car_Button_Event "/CarButtonEvent"
#endif

// camel订阅的rfid信息
#ifndef camel_sub_topic_rfid
#define camel_sub_topic_rfid "/agv_rfid"
#endif

// camel订阅的磁传感器信息
#ifndef camel_sub_topic_gpio
#define camel_sub_topic_gpio "/agv_gpio"
#endif

// camel发布的查询告警信息的服务
#ifndef camel_service_alarm_info
#define camel_service_alarm_info "AlarmInfo"
#endif

// camel发布的查询告警信息的服务
#ifndef camel_service_position_info
#define camel_service_position_info "PositionInfo"
#endif

// camel发布的查询告警信息的服务
#ifndef camel_service_task_info
#define camel_service_task_info "TaskInfo"
#endif

// camel发布的查询告警信息的服务
#ifndef camel_service_running_info
#define camel_service_running_info "RunningInfo"
#endif

// camel发布的查询告警信息的服务
#ifndef camel_service_system_info
#define camel_service_system_info "SystemInfo"
#endif

// camel给控制的多车组队指令
#ifndef camel_pub_topic_Camel2Collaborative
#define camel_pub_topic_Camel2Collaborative "/Camel2Collaborative"
#endif

// 控制给camel的组队后的数据
#ifndef camel_pub_topic_Collaborative2Camel
#define camel_pub_topic_Collaborative2Camel "/Collaborative2Camel"
#endif

// 切换地图
#ifndef camel_service_load_map
#define camel_service_load_map "/load_map"
#endif

// camel向导航申请开始(关闭)画路线
#ifndef camel_service_draw_flag
#define camel_service_draw_flag "/camel/drawAgvPathFlag"
#endif

// camel订阅导航发布的路线
#ifndef camel_sub_topic_agv_path
#define camel_sub_topic_agv_path "/z_localizer/agv_path"
#endif

// camel向导航申请增加路线
#ifndef camel_service_add_route
#define camel_service_add_route "/camel/addRoute"
#endif

// camel向导航查询测距光电值
#ifndef camel_service_check_pallet_dis
#define camel_service_check_pallet_dis "/camel/CheckPalletDis"
#endif

// camel向idoo查询测距光电值
#ifndef camel_service_idoo_check_pallet_dis
#define camel_service_idoo_check_pallet_dis "/idoo/CheckPalletDis"
#endif

// camel申请获取是否可以取放货
#ifndef camel_service_check_position
#define camel_service_check_position "/check_position"
#endif


// camel向plc申请硬件信息
#ifndef camel_service_car_hardware_details
#define camel_service_car_hardware_details "/CarHardwareDetails"
#endif

//camel向末端相机申请库位是否可放货
#ifndef camel_service_check_space
#define camel_service_check_space "/camel/CheckSpace"
#endif

// camel向导航请求qr_station_server
#ifndef camel_service_qr_station_info
#define camel_service_qr_station_info "/qr_station_server"
#endif

namespace camelrosmsg
{
    typedef struct tag_carpose
    {
        tag_carpose()
        {
            memset(this, 0, sizeof(*this));
        }

        tag_carpose(const tag_carpose &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, sizeof(tag_carpose));
            }
        }

        tag_carpose &operator=(const tag_carpose &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, sizeof(tag_carpose));
            }
            return *this;
        }

        ~tag_carpose() {}

        double x;
        double y;
        double z;
        double theta;
    } CarPose;

    typedef struct tag_AGVGoal
    {
        tag_AGVGoal()
        {
            memset(this, 0, sizeof(*this));
        }

        tag_AGVGoal(const tag_AGVGoal &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, sizeof(tag_AGVGoal));
            }
        }

        tag_AGVGoal &operator=(const tag_AGVGoal &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, sizeof(tag_AGVGoal));
            }
            return *this;
        }

        ~tag_AGVGoal() {}

        CarPose pose;
        double v;
        double w;
    } AGVGoal;

	typedef struct tag_camel_2_plc_notify
	{
		tag_camel_2_plc_notify() { memset(this, 0, sizeof *this); }
		tag_camel_2_plc_notify(const tag_camel_2_plc_notify &rhs)
		{
			if (this != &rhs)
			{
				memcpy(this, &rhs, sizeof(tag_camel_2_plc_notify));
			}
		}

		unsigned long long int   ObstacleArea; // 障碍物区域
		unsigned short u16BrakeSignal;	 // 刹车 0:不刹车 1:刹车
		unsigned short u16CargoType;	 // 货物类型  0:空车 1：轻载、2：重载；
		unsigned short u16CargoWeight;  // 载物重量 单位(kg)
		unsigned short u16Enable;	     // 1:使能 0:不使能
		unsigned short u16CamelAlarm;   // 级别报警(1, 2, 3, 4)
		unsigned short u16TurnType;   // 转弯类型，0直线，1车头朝前左转，2车头朝前右转，3叉子朝前左转，4叉子朝前右转
		unsigned short u16ForkLight;   // 叉头光电 0:不打开 1：打开  // 2024-03-19 沈工  1 屏蔽叉头光电 
        bool           beginStart;      //一键启动
        bool           musicPause;    // 音乐暂停
        bool           finishMusic;   // 音乐结束
	}CAMEL_2_PLC_NOTIFY;

	typedef struct tag_PLC_Action
	{
		tag_PLC_Action()
		{
			memset(this, 0, sizeof(*this));
		}

		tag_PLC_Action(const tag_PLC_Action &rhs)
		{
			if (this != &rhs)
			{
				memcpy(this, &rhs, sizeof(tag_PLC_Action));
			}
		}

		tag_PLC_Action &operator=(const tag_PLC_Action &rhs)
		{
			if (this != &rhs)
			{
				memcpy(this, &rhs, sizeof(tag_PLC_Action));
			}
			return *this;
		}

		~tag_PLC_Action() {}

		unsigned short u16Type;			 // 动作类型
		unsigned char  u8Mode;				 // 动作模式 0: 无 1:定点  2: 编解码使用
	} CAMEL_2_PLC_ACTION;

	typedef struct tag_c_2_plc_notify
	{
		tag_c_2_plc_notify() { memset(this, 0, sizeof *this); }
		tag_c_2_plc_notify(const tag_c_2_plc_notify &rhs)
		{
			if (this != &rhs)
			{
				memcpy(this, &rhs, sizeof(tag_c_2_plc_notify));
			}
		}

		float	fVx; // 期望速度Vx
		float	fVy; // 期望速度Vy
		float	fW;  // 期望角速度W
		unsigned char      u8Flag;  // 圆弧标记
	} C_2_PLC_NOTIFY;

    typedef struct tag_PolygonPoint
    {
        tag_PolygonPoint()
        {
            memset(this, 0, sizeof(*this));
        }

        tag_PolygonPoint(const tag_PolygonPoint &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, sizeof(tag_PolygonPoint));
            }
        }

        tag_PolygonPoint &operator=(const tag_PolygonPoint &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, sizeof(tag_PolygonPoint));
            }
            return *this;
        }

        ~tag_PolygonPoint() {}

        float x;
        float y;
        float z;
    } PolygonPoint;

    typedef std::vector<PolygonPoint> PolygonArea;

    typedef struct tag_ResultFromA
    {
        tag_ResultFromA()
        {
            memset(this, 0, ((char *)&(this->frame) - (char *)this));
            frame = "";
            ref_frame = "";
        }

        tag_ResultFromA(const tag_ResultFromA &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, ((char *)&rhs.frame - (char *)&rhs));
                this->frame = rhs.frame;
                this->ref_frame = rhs.ref_frame;
            }
        }

        tag_ResultFromA &operator=(const tag_ResultFromA &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, ((char *)&rhs.frame - (char *)&rhs));
                this->frame = rhs.frame;
                this->ref_frame = rhs.ref_frame;
            }
            return *this;
        }

        ~tag_ResultFromA() {}

        int stamp_secs;
        int stamp_nsecs;
        int seq_id;
        int loc_state;
        float certainty;
        CarPose pose;
        std::string frame;
        std::string ref_frame;
    } ResultFromA;

    typedef struct tag_TerminalRequest
    {
        tag_TerminalRequest()
        {
            memset(this, 0, sizeof(*this));
        }

        tag_TerminalRequest(const tag_TerminalRequest &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, sizeof(tag_TerminalRequest));
            }
        }

        tag_TerminalRequest &operator=(const tag_TerminalRequest &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, sizeof(tag_TerminalRequest));
            }
            return *this;
        }

        ~tag_TerminalRequest() {}

        bool flag;
        int target_type;
        CarPose pose;
    } TerminalRequest;

    typedef struct tag_TerminalResponse
    {
        tag_TerminalResponse()
        {
            error_code = 0;
            error_msg = "";
        }

        tag_TerminalResponse(const tag_TerminalResponse &rhs)
        {
            if (this != &rhs)
            {
                this->error_code = rhs.error_code;
                this->error_msg = rhs.error_msg;
            }
        }

        tag_TerminalResponse &operator=(const tag_TerminalResponse &rhs)
        {
            if (this != &rhs)
            {
                this->error_code = rhs.error_code;
                this->error_msg = rhs.error_msg;
            }
            return *this;
        }

        ~tag_TerminalResponse() {}

        int error_code;
        std::string error_msg;
    } TerminalResponse;

    typedef struct tag_LoadMapRequest
    {
        bool initial_pose;
        double x;
        double y;
        double yaw;
        std::string map_name;

        tag_LoadMapRequest() : initial_pose(false), x(0.0), y(0.0), yaw(0.0) {}
        tag_LoadMapRequest(double x, double y, double yaw, const std::string &map)
            : initial_pose(true), x(x), y(y), yaw(yaw), map_name(map)
        {
        }

    } LoadMapRequest;

    typedef struct tag_DrawFlagRequest
    {
        bool flag;
        tag_DrawFlagRequest() : flag(false) {}
        tag_DrawFlagRequest(bool flag) : flag(flag) {}

    } DrawFlagRequest;

     typedef struct tag_CheckSpaceRequest
    {
        bool flag;
        tag_CheckSpaceRequest() : flag(false) {}
        tag_CheckSpaceRequest(bool flag) : flag(flag) {}

    } CheckSpaceRequest;

    typedef struct tag_ServiceResponse
    {
        bool success;
        std::string err_msg;

        tag_ServiceResponse() : success(false) {}
    } ServiceResponse;

    typedef struct tag_CamelPLCROSMsg
    {
        tag_CamelPLCROSMsg()
        {
            memset(this, 0, sizeof(*this));
        }

        tag_CamelPLCROSMsg(const tag_CamelPLCROSMsg &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, sizeof(tag_CamelPLCROSMsg));
            }
        }

        tag_CamelPLCROSMsg &operator=(const tag_CamelPLCROSMsg &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, sizeof(tag_TerminalRequest));
            }
            return *this;
        }

        ~tag_CamelPLCROSMsg() {}

        char data[camel_plc_msg_max_length];
        int length;
    } CamelPLCRosMsg;

typedef struct tag_CamelGetPLCDirectDataMsg
	{
		tag_CamelGetPLCDirectDataMsg()
		{
			memset(this, 0, sizeof(*this));
		}

		tag_CamelGetPLCDirectDataMsg(const tag_CamelGetPLCDirectDataMsg &rhs)
		{
			if (this != &rhs)
			{
				memcpy(this, &rhs, sizeof(tag_CamelGetPLCDirectDataMsg));
			}
		}

		tag_CamelGetPLCDirectDataMsg &operator=(const tag_CamelGetPLCDirectDataMsg &rhs)
		{
			if (this != &rhs)
			{
				memcpy(this, &rhs, sizeof(tag_CamelGetPLCDirectDataMsg));
			}
			return *this;
		}

		~tag_CamelGetPLCDirectDataMsg() {}

		bool InManualCharging;
		bool SemiAutomaticMode;
	} CamelGetPLCData;

    typedef struct tag_CamelOdom
    {
        tag_CamelOdom()
        {
            memset(this, 0, sizeof(*this));
        }

        tag_CamelOdom(const tag_CamelOdom &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, sizeof(tag_CamelOdom));
            }
        }

        tag_CamelOdom &operator=(const tag_CamelOdom &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, sizeof(tag_CamelOdom));
            }
            return *this;
        }

        ~tag_CamelOdom() {}

        CarPose pose;
        double vx;
        double vy;
        double w;
    } CamelOdom;

    typedef struct tag_CamelCarState
    {
        tag_CamelCarState()
        {
            memset(this, 0, ((char *)&(this->currentStation) - (char *)this));
            currentStation = "";
        }

        tag_CamelCarState(const tag_CamelCarState &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, ((char *)&rhs.currentStation - (char *)&rhs));
                this->currentStation = rhs.currentStation;
            }
        }

        tag_CamelCarState &operator=(const tag_CamelCarState &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, ((char *)&rhs.currentStation - (char *)&rhs));
                this->currentStation = rhs.currentStation;
            }
            return *this;
        }

        ~tag_CamelCarState() {}

        float batteryPercent;
        uint8_t chargingState; // 0: 未充电 1: 充电中 2: 充电异常
        uint8_t ctrlMode;      //
        uint8_t ctrlStatus;    //
        bool hasRoute;
        bool cargoState;
        bool mapNotMatch;
        bool onRouteEnd;
        std::string currentStation;
        std::string carName;
    } CamelCarState;

    typedef struct tag_CamelQRInfo
    {
        tag_CamelQRInfo()
        {
            memset(this, 0, sizeof(*this));
        }

        tag_CamelQRInfo(const tag_CamelQRInfo &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, sizeof(tag_CamelQRInfo));
            }
        }

        tag_CamelQRInfo &operator=(const tag_CamelQRInfo &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, sizeof(tag_CamelQRInfo));
            }
            return *this;
        }

        ~tag_CamelQRInfo() {}

        uint32_t id;
        int16_t offset_x;
        int16_t offset_y;
        float angle;
    } CamelQRInfo;

    typedef struct tag_CamelROSErrorCodesInfo
    {
        tag_CamelROSErrorCodesInfo()
        {
            code_id = 0;
            params.clear();
        }

        tag_CamelROSErrorCodesInfo(const tag_CamelROSErrorCodesInfo &rhs)
        {
            if (this != &rhs)
            {
                code_id = rhs.code_id;
                params = rhs.params;
            }
        }

        tag_CamelROSErrorCodesInfo &operator=(const tag_CamelROSErrorCodesInfo &rhs)
        {
            if (this != &rhs)
            {
                code_id = rhs.code_id;
                params = rhs.params;
            }
            return *this;
        }

        ~tag_CamelROSErrorCodesInfo() {}
        int code_id;
        std::vector<std::string> params;
    } CamelROSErrorCodesInfo;

    typedef struct tag_CamelROSAlarmInfo
    {
        tag_CamelROSAlarmInfo()
        {
            source = "";
            err_codes.clear();
        }

        tag_CamelROSAlarmInfo(const tag_CamelROSAlarmInfo &rhs)
        {
            if (this != &rhs)
            {
                source = rhs.source;
                err_codes = rhs.err_codes;
            }
        }

        tag_CamelROSAlarmInfo &operator=(const tag_CamelROSAlarmInfo &rhs)
        {
            if (this != &rhs)
            {
                source = rhs.source;
                err_codes = rhs.err_codes;
            }
            return *this;
        }

        ~tag_CamelROSAlarmInfo() {}

        std::string source;
        std::vector<CamelROSErrorCodesInfo> err_codes;
    } CamelROSAlarmInfo;

    typedef struct tag_ActionEntry
    {
        using KeyValue = std::pair<std::string, std::string>;
        using Params = std::vector<KeyValue>;
        enum ActionType
        {
            AT_STARTBAG = 0,
            AT_ENDBAG = 1
        };

        ActionType action_type;
        std::string ns;
        unsigned action_id;
        Params params;

        tag_ActionEntry()
            : action_type(AT_STARTBAG), action_id(0)
        {
        }
    } ActionEntry;

    typedef struct tag_Header
    {
        int stamp_secs;
        int stamp_nsecs;
        int seq_id;
        std::string frame;
        tag_Header()
            : stamp_secs(0), stamp_nsecs(0), seq_id(0)
        {
        }
    } Header;

    typedef struct tag_ActionList
    {
        using Actions = std::vector<ActionEntry>;
        Actions action_list;
    } ActionList;

    typedef struct tag_ActionFeedback
    {
        enum ActionResult
        {
            AR_SUCCESS = 0,
            AR_FAIL = 1,
            AR_NOT_IMPLEMEMT = 2,
            AR_INVALID_ARGUMENT = 3
        };

        Header header;
        ActionResult result;
        unsigned action_id;
        std::string err_msg;

        tag_ActionFeedback()
            : result(AR_SUCCESS), action_id(0)
        {
        }
    } ActionFeedback;

    typedef struct tag_AgvAlarm
    {
        tag_AgvAlarm()
        {
            memset(this, 0, ((char *)&(this->message) - (char *)this));
            message = "";
        }

        tag_AgvAlarm(const tag_AgvAlarm &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, ((char *)&rhs.message - (char *)&rhs));
                this->message = rhs.message;
            }
        }

        tag_AgvAlarm &operator=(const tag_AgvAlarm &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, ((char *)&rhs.message - (char *)&rhs));
                this->message = rhs.message;
            }
            return *this;
        }

        ~tag_AgvAlarm() {}

        uint32_t id;
        uint32_t source;
        uint32_t level;
        uint32_t status;
        std::string message;
    } AgvAlarm;

    typedef struct tag_QrErrorArrived
    {
        tag_QrErrorArrived()
        {
            memset(this, 0, sizeof(*this));
        }

        tag_QrErrorArrived(const tag_QrErrorArrived &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, sizeof(tag_QrErrorArrived));
            }
        }

        tag_QrErrorArrived &operator=(const tag_QrErrorArrived &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, sizeof(tag_QrErrorArrived));
            }
            return *this;
        }

        ~tag_QrErrorArrived() {}

        bool arrive;
        bool dataValid;
        float x;
        float y;
        float theta;
    } QrErrorArrived;

    typedef struct tag_CarButtonEvent
    {
        uint8_t onResetPress;
        uint8_t onResetRelease;
        uint8_t onResetLongPress;

        tag_CarButtonEvent()
        {
            memset(this, 0, sizeof(tag_CarButtonEvent));
        }
        ~tag_CarButtonEvent() {}

    } CarButtonEvent;

    typedef struct tag_Collaborative2Camel
    {
        uint32_t cooperationStatus; // 1完成组队;0未完成
        float colla_x;
        float colla_y;
        float colla_theta;
        tag_Collaborative2Camel()
        {
            memset(this, 0, sizeof(tag_Collaborative2Camel));
        }
        ~tag_Collaborative2Camel() {}
    } Collaborative2Camel;

    typedef struct tag_Camel2Collaborative
    {
        uint32_t startCooperation; // 1下发组队指令;0未下发
        std::string headCarIp;
        std::string rearCarIp;
        uint32_t reach_mission;       // 1到达任务终点;0未到达
        uint32_t endCooperation;      // 1下发删除协同组队指令;0未下发
        uint32_t masterSlaveIdentity; // 1为主车;2从车
        tag_Camel2Collaborative()
            : startCooperation(0), reach_mission(0), endCooperation(0), masterSlaveIdentity(0)
        {
        }
        ~tag_Camel2Collaborative() {}
    } Camel2Collaborative;
    typedef struct tag_RfidInfo
    {
        uint32_t rfid;

        tag_RfidInfo()
        {
            memset(this, 0, sizeof(tag_RfidInfo));
        }
        ~tag_RfidInfo() {}

    } RfidInfo;

	typedef struct tag_AGVStatusInfo
	{
		tag_AGVStatusInfo()
		{
			distance_to_target = 0.0f;
			current_x = 0.0f;
			current_y = 0.0f;
			current_theta = 0.0f;
			has_route = false;
			has_task = false;
			current_station = "";
			station_type = "";
		}

		tag_AGVStatusInfo(const tag_AGVStatusInfo &rhs)
		{
			if (this != &rhs)
			{
				distance_to_target = rhs.distance_to_target;
				current_x = rhs.current_x;
				current_y = rhs.current_y;
				current_theta = rhs.current_theta;
				has_route = rhs.has_route;
				has_task = rhs.has_task;
				current_station = rhs.current_station;
				station_type = rhs.station_type;
			}
		}

		tag_AGVStatusInfo &operator=(const tag_AGVStatusInfo &rhs)
		{
			if (this != &rhs)
			{
				distance_to_target = rhs.distance_to_target;
				current_x = rhs.current_x;
				current_y = rhs.current_y;
				current_theta = rhs.current_theta;
				has_route = rhs.has_route;
				has_task = rhs.has_task;
				current_station = rhs.current_station;
				station_type = rhs.station_type;
			}
			return *this;
		}

		~tag_AGVStatusInfo() {}

		float distance_to_target;  // 到目标点距离
		float current_x;          // 当前x坐标
		float current_y;          // 当前y坐标
		float current_theta;      // 当前角度
		bool has_route;          // 是否有路线
		bool has_task;           // 是否有任务
		std::string current_station;  // 当前站点
		std::string station_type;     // 站点类型
	} AGVStatusInfo;

    typedef struct tag_GpioInfo
    {
        uint16_t magnetic_front;
        uint16_t magnetic_back;
        uint16_t magnetic_break;

        tag_GpioInfo()
        {
            memset(this, 0, sizeof(tag_GpioInfo));
        }
        ~tag_GpioInfo() {}

    } GpioInfo;

    typedef struct tag_PositionInfo
    {
        tag_PositionInfo()
        {
            memset(this, 0, ((char *)&(this->currentStation) - (char *)this));
            currentStation = "";
        }

        tag_PositionInfo(const tag_PositionInfo &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, ((char *)&rhs.currentStation - (char *)&rhs));
                this->currentStation = rhs.currentStation;
            }
        }

        tag_PositionInfo &operator=(const tag_PositionInfo &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, ((char *)&rhs.currentStation - (char *)&rhs));
                this->currentStation = rhs.currentStation;
            }
            return *this;
        }

        ~tag_PositionInfo() {}

        float x;
        float y;
        float theta;
        float certainty;
        std::string currentStation;
    } AGVPositionInfo;

    typedef struct tag_AGVRunningInfo
    {
        int32_t mode;
        int32_t status;
        float vxReal;
        float vyReal;
        float wReal;
        float vxExp;
        float vyExp;
        float wExp;
        float battery;
        float chargeState;
        bool cargo;

        tag_AGVRunningInfo()
        {
            memset(this, 0, sizeof(tag_AGVRunningInfo));
        }
        ~tag_AGVRunningInfo() {}

    } AGVRunningInfo;

    typedef struct tag_AGVSystemInfo
    {
        tag_AGVSystemInfo()
        {
            memset(this, 0, ((char *)&(this->carName) - (char *)this));
            carName = "";
            version = "";
            mapInfo = "";
            ipAddr = "";
            mac = "";
        }

        tag_AGVSystemInfo(const tag_AGVSystemInfo &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, ((char *)&rhs.carName - (char *)&rhs));
                this->carName = rhs.carName;
                this->version = rhs.version;
                this->mapInfo = rhs.mapInfo;
                this->ipAddr = rhs.ipAddr;
                this->mac = rhs.mac;
            }
        }

        tag_AGVSystemInfo &operator=(const tag_AGVSystemInfo &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, ((char *)&rhs.carName - (char *)&rhs));
                this->carName = rhs.carName;
                this->version = rhs.version;
                this->mapInfo = rhs.mapInfo;
                this->ipAddr = rhs.ipAddr;
                this->mac = rhs.mac;
            }
            return *this;
        }

        ~tag_AGVSystemInfo() {}

        float cpuUsage;
        float memUsage;
        std::string carName;
        std::string version;
        std::string mapInfo;
        std::string ipAddr;
        std::string mac;
    } AGVSystemInfo;

    typedef struct tag_AGVTaskInfo
    {
        tag_AGVTaskInfo()
        {
            route = "";
            targetStation = "";
        }

        tag_AGVTaskInfo(const tag_AGVTaskInfo &rhs)
        {
            if (this != &rhs)
            {
                this->route = rhs.route;
                this->targetStation = rhs.targetStation;
            }
        }

        tag_AGVTaskInfo &operator=(const tag_AGVTaskInfo &rhs)
        {
            if (this != &rhs)
            {
                this->route = rhs.route;
                this->targetStation = rhs.targetStation;
            }
            return *this;
        }

        ~tag_AGVTaskInfo() {}

        std::string route;
        std::string targetStation;
    } AGVTaskInfo;

    typedef struct tag_Path
    {
        tag_Path()
        {
            memset(this, 0, sizeof(*this));
        }

        tag_Path(const tag_Path &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, sizeof(tag_Path));
            }
        }

        tag_Path &operator=(const tag_Path &rhs)
        {
            if (this != &rhs)
            {
                memcpy(this, &rhs, sizeof(tag_Path));
            }
            return *this;
        }

        ~tag_Path() {}

        bool direct;
        CarPose start_pose;
        CarPose end_pose;
        float radius;
        bool startIsTaskPoint;
        long start_id;
        bool endIsTaskPoint;
        long end_id;
    } Path;

    typedef struct tag_AgvPaths
    {
        std::vector<tag_Path> paths;

    } AGVPaths;

    typedef struct tag_AddRouteResponse
    {
        tag_AddRouteResponse()
            : success(false)
        {
        }

        bool success;
        std::string err_msgs;
        AGVPaths paths;
    } AddRouteResponse;

    typedef struct tag_CheckPalletDis
    {
        tag_CheckPalletDis()
            : distance(0)
        {
        }
        unsigned distance;
    } CheckPalletDis;

    typedef struct tag_CarHardwareDetails
    {
        tag_CarHardwareDetails()
            : leftOpticalSignal(-1), rightOpticalSignal(-1),forkGear(-1)
        {
        }

        int leftOpticalSignal;
        int rightOpticalSignal;
        int forkGear; 
    } CarHardwareDetails;

	typedef struct tag_CargoAttribute
	{
		tag_CargoAttribute()
			: hasCargo(false), cargoType(""), cargoLength(-1), cargoWidth(-1), cargoHeight(-1), cargoCenterToMotionCenterDist(-1), cargoWeight(-1)
		{
		}

		bool			hasCargo;
		std::string		cargoType;
		int32_t			cargoLength;
		int32_t			cargoWidth;
		int32_t			cargoHeight;
		int32_t			cargoCenterToMotionCenterDist;
		float			cargoWeight;
	} CargoAttribute;
	
	typedef struct tag_ActiveSafeEnable
	{
		tag_ActiveSafeEnable()
			: taskStatus(0), isActiveSafetyOn(false)
		{
		}

		int8_t			taskStatus;
		bool			isActiveSafetyOn;

	} ActiveSafeEnable;

    typedef struct tag_QrStationInfoRequest
    {
		std::string station_and_qr_info;
		tag_QrStationInfoRequest() : station_and_qr_info("") {}
        tag_QrStationInfoRequest(std::string station_and_qr_info) : station_and_qr_info(station_and_qr_info) {}
        
    } QrStationInfoRequest;

    class CSubscribeHandle
    {
    public:
        CSubscribeHandle(){};
        virtual ~CSubscribeHandle(){};
        virtual void LocalizationResultHandle(const ResultFromA &msg) = 0;
        virtual long OnGetRosPlcCycleNotify(void *pData, int data_len) = 0; // ros plc 周期发布数据
        virtual long OnGetRosPlcActionRsp(void *pData, int data_len) = 0;   // ros plc 动作响应
	    virtual void OnGetRosPlcDirectdata(const camelrosmsg::CamelGetPLCData &pmsg) = 0;//周期性获取plc数据
        virtual void OnGetC2CamelMsg(void *pMsg) = 0;
        virtual void OnReceiveAvoidOperationRequest(const uint32_t request, int32_t &response) = 0;
        virtual void OnGetCarState(CamelCarState &state) = 0;
        virtual void OnGetQRCameraCheckMsg(const CamelQRInfo &msg) = 0;
        virtual void OnGetQRCameraMsg(const CamelQRInfo &msg) = 0;
        virtual void OnGetAlarmsMsg(const CamelROSAlarmInfo &msg) = 0;
        virtual void OnGetActionFeedback(const ActionFeedback &msg) = 0;
        virtual void OnGetMarkerLR(const ResultFromA &msg) = 0;
        virtual void OnGetQrErrorArrived(const QrErrorArrived &msg) = 0;
        virtual void OnGetRfidInfo(const RfidInfo &msg) = 0;
        virtual void OnGetGpioInfo(const GpioInfo &msg) = 0;
        virtual void GetAlarmInfo(std::vector<AgvAlarm> &alarm) = 0;
        virtual void GetPositionInfo(AGVPositionInfo &info) = 0;
        virtual void GetRunningInfo(AGVRunningInfo &info) = 0;
        virtual void GetSystemInfo(AGVSystemInfo &info) = 0;
        virtual void GetTaskInfo(AGVTaskInfo &info) = 0;
        virtual void OnCollaborative2Camel(const Collaborative2Camel &msg) = 0;
        virtual void OnGetAgvPath(const AGVPaths &paths) = 0;
    };

    class CRosPublish
    {
    public:
        CRosPublish(){};
        virtual ~CRosPublish(){};
        virtual void Publish(const std::string &topic, void *pmsg) = 0;
        virtual void BindSubscribe(CSubscribeHandle *subs) = 0;
        virtual bool CallService(const std::string &service, void *prequest, void *presponse) = 0;
        virtual void Release() = 0;
    };

    class CRosConfigHandle
	{
	public:
        virtual ~CRosConfigHandle(){};
        virtual bool HasConfig(const std::string& key) = 0;
		virtual bool GetConfig(const std::string& key, std::string& value) = 0;
		virtual bool SetConfig(const std::string& key, const std::string& value) = 0;
        virtual bool GetConfig(const std::string& key, int& value) = 0;
		virtual bool SetConfig(const std::string& key, const int& value) = 0;
        virtual bool GetConfig(const std::string& key, bool& value) = 0;
		virtual bool SetConfig(const std::string& key, const bool& value) = 0;
        virtual bool GetConfig(const std::string& key, float& value) = 0;
		virtual bool SetConfig(const std::string& key, const float& value) = 0;
        virtual bool GetConfig(const std::string& key, std::map<std::string, bool>& maps) = 0;
	};

    class CRosFactory
    {
    public:
        CRosFactory(){};
        ~CRosFactory(){};
        static CRosPublish *CreatePublish();
        static void DeletePublish(CRosPublish *);
        static CRosConfigHandle* GetRosConfig();
    };

}

#endif /*__ROS_ADAPTER_H__*/
