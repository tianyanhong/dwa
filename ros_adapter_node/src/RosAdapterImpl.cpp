#include <signal.h>
#include "tf/tf.h"
#include "RosAdapterImpl.h"

using namespace Camel::Controller;

void CamelRosSigintHandler(int sig)
{
	ros::shutdown();
	exit(0);
};

namespace camelrosmsg
{
	void CRosAdapterImpl::Publish(const std::string &topic, void *pmsg)
	{
		if (pmsg == nullptr)
		{
			return;
		}

		if (map_pub_handle_.count(topic) == 0)
		{
			return;
		}

		ros::Publisher pub_handle_ = map_pub_handle_[topic];

		PublishTopic pub_topic = map_pub_type_[topic];
		switch (pub_topic)
		{
		case PT_AGVGoal:
		{
			AGVGoal *ptempMsg = (AGVGoal *)pmsg;
			ros_adapter_node::Navigation rosmsg;
			EncodeRosAgvGoalMsg(ptempMsg, rosmsg);
			pub_handle_.publish(rosmsg);
		}
		break;

		case PT_ProhibitedArea:
		case PT_AccessibleArea:
		{
			PolygonArea *parea = (PolygonArea *)pmsg;
			geometry_msgs::Polygon areamsg;
			for (auto point : *parea)
			{
				geometry_msgs::Point32 rospoint;
				rospoint.x = point.x;
				rospoint.y = point.y;
				rospoint.z = point.z;
				areamsg.points.push_back(rospoint);
			}
			pub_handle_.publish(areamsg);
		}
		break;

		case PT_InitPose:
		{
			CarPose *ppose = (CarPose *)pmsg;
			geometry_msgs::PoseWithCovarianceStamped rosmsg;
			EncodeRosInitPoseMsg(ppose, rosmsg);
			pub_handle_.publish(rosmsg);
		}
		break;

		case PT_Camel2PlcNotify:
		case PT_Camel2PlcActionReq:
		{
			CamelPLCRosMsg *ptoPlc = (CamelPLCRosMsg *)pmsg;
			ros_adapter_node::camel_to_plc rosmsg;
			rosmsg.length = ptoPlc->length;
			memcpy(rosmsg.camel_to_plc_message.c_array(), ptoPlc->data, ptoPlc->length);

			pub_handle_.publish(rosmsg);
		}
		break;
		case PT_Camel2Plc:
		{
			CAMEL_2_PLC_NOTIFY *ptoPlc = (CAMEL_2_PLC_NOTIFY *)pmsg;
			ros_adapter_node::camel_to_plc_direct rosmsg;
			rosmsg.u64ObstacleArea = ptoPlc->ObstacleArea;
			rosmsg.u16BrakeSignal = ptoPlc->u16BrakeSignal;
			rosmsg.u16CargoType = ptoPlc->u16CargoType;
			rosmsg.u16CargoWeight = ptoPlc->u16CargoWeight;
			rosmsg.u16Enable = ptoPlc->u16Enable;
			rosmsg.u16CamelAlarm = ptoPlc->u16CamelAlarm;
			rosmsg.u16TurnType = ptoPlc->u16TurnType;
			rosmsg.u16ForkLight = ptoPlc->u16ForkLight;
			rosmsg.beginStart = ptoPlc->beginStart;
			rosmsg.musicPause = ptoPlc->musicPause;
			rosmsg.finishMusic = ptoPlc->finishMusic;

			pub_handle_.publish(rosmsg);
		}
		break;
		case PT_Odom:
		{
			CamelOdom *pcamelOdom = (CamelOdom *)pmsg;
			nav_msgs::Odometry rosOdom;
			EncodeRosOdomMsg(pcamelOdom, rosOdom);
			pub_handle_.publish(rosOdom);
		}
		break;

		case PT_Camel2Cbase:
		{
			CINBASE *ptempMsg = (CINBASE *)pmsg;
			ros_adapter_node::State_msg rosmsg;
			EncodeRosCamel2CBaseMsg(ptempMsg, rosmsg);
			pub_handle_.publish(rosmsg);
		}
		break;

		case PT_Camel2Cpoints:
		{
			std::vector<CPOINT> *ptempMsg = (std::vector<CPOINT> *)pmsg;
			ros_adapter_node::Point_vector_msg rosmsg;
			EncodeRosCamel2CPointsMsg(ptempMsg, rosmsg);
			pub_handle_.publish(rosmsg);
		}
		break;

		case PT_Camel2Cterminal:
		{
			RELATIVE_POS *ptempMsg = (RELATIVE_POS *)pmsg;
			ros_adapter_node::Terminal_msg rosmsg;
			EncodeRosCamel2CTerminalMsg(ptempMsg, rosmsg);
			pub_handle_.publish(rosmsg);
		}
		break;
		
		case PT_Camel2CterminalConfig:
		{
			TERMINAL_CONFIG *ptempMsg = (TERMINAL_CONFIG *)pmsg;
			ros_adapter_node::TerminalConfig rosmsg;
			EncodeRosCamel2CTerminalConfig(ptempMsg, rosmsg);
			pub_handle_.publish(rosmsg);
		}
		break;

		case PT_Camel2PlcStick:
		{
			AGVGoal *ptempMsg = (AGVGoal *)pmsg;
			sensor_msgs::Joy rosmsg;
			rosmsg.axes.push_back(ptempMsg->w);
			rosmsg.axes.push_back(0);
			rosmsg.axes.push_back(0);
			rosmsg.axes.push_back(ptempMsg->v);
			pub_handle_.publish(rosmsg);
		}
		break;

		case PT_Camel23DCargoAttribute:
		{
			CargoAttribute *ptempMsg = (CargoAttribute *)pmsg;
			ros_adapter_node::Cargo_msg rosmsg;
			rosmsg.has_cargo = ptempMsg->hasCargo;
			rosmsg.dis_to_center = ptempMsg->cargoCenterToMotionCenterDist;
			rosmsg.cargo_height = ptempMsg->cargoHeight;
			rosmsg.cargo_length = ptempMsg->cargoLength;
			rosmsg.cargo_name = ptempMsg->cargoType;
			rosmsg.weight = ptempMsg->cargoWeight;
			rosmsg.cargo_width = ptempMsg->cargoWidth;

			pub_handle_.publish(rosmsg);
		}
		break;

		case PT_Camel23DActiveSafeEnable:
		{
			ActiveSafeEnable *ptempMsg = (ActiveSafeEnable *)pmsg;
			ros_adapter_node::Active_safety_enable_msg rosmsg;
			rosmsg.taskStatus = ptempMsg->taskStatus;
			rosmsg.isActiveSafetyOn = ptempMsg->isActiveSafetyOn;

			pub_handle_.publish(rosmsg);
		}
		break;

		case PT_ActionList:
		{
			ActionList *pActions = (ActionList *)pmsg;
			ros_adapter_node::ActionList rosmsg;
			EncodeActionList(pActions, rosmsg);
			pub_handle_.publish(rosmsg);
		}
		break;

		case PT_AgvAlarm:
		{
			std::vector<AgvAlarm> *ptempMsg = (std::vector<AgvAlarm> *)pmsg;
			ros_adapter_node::AgvAlarmSet rosmsg;
			EncodeAgvAlarm(ptempMsg, rosmsg);
			pub_handle_.publish(rosmsg);
		}
		break;

		case PT_AgvStatus:
		{
			CamelCarState *ptempMsg = (CamelCarState *)pmsg;
			ros_adapter_node::AgvStatus rosmsg;
			EncodeAgvStatus(ptempMsg, rosmsg);
			pub_handle_.publish(rosmsg);
		}
		break;

		case PT_CarButtonEvent:
		{
			CarButtonEvent *pEvent = (CarButtonEvent *)pmsg;
			ros_adapter_node::CarButtonEvent rosmsg;
			EncodeCarButtonEvent(pEvent, rosmsg);
			pub_handle_.publish(rosmsg);
		}
		break;

		case PT_Camel2Collaborative:
		{
			Camel2Collaborative *pData = (Camel2Collaborative *)pmsg;
			ros_adapter_node::Camel2Collaborative rosmsg;
			EncodeCamel2Collaborative(pData, rosmsg);
			pub_handle_.publish(rosmsg);
		}
		break;

		case PT_AgvStatusInfo:
        {
            AGVStatusInfo* ptempMsg = (AGVStatusInfo*)pmsg;
            ros_adapter_node::AGVStatusInfo rosmsg;
            EncodeAgvStatusInfo(ptempMsg, rosmsg);
            pub_handle_.publish(rosmsg);
        }
        break;

		default:
		{
			// do nothing
		}
		}
	}

	void CRosAdapterImpl::BindSubscribe(CSubscribeHandle *subs)
	{
		if (subs == nullptr)
		{
			return;
		}
		psubhandle = subs;
	}

	void CRosAdapterImpl::Init(ros::NodeHandle n)
	{
		nh_ = n;
		/* 注意：注册订阅/发布的话题或服务，一定要把对应的句柄保存在类成员对应的map中，
		 * 否则句柄作为局部变量被释放后，对应的话题或服务也就释放了	*/

		// camel发布的话题
		ros::Publisher pub_handle_goal = nh_.advertise<ros_adapter_node::Navigation>(camel_pub_topic_agv_goal, 10);
		map_pub_handle_.insert(std::pair<std::string, ros::Publisher>(camel_pub_topic_agv_goal, pub_handle_goal));
		map_pub_type_.insert(std::pair<std::string, PublishTopic>(camel_pub_topic_agv_goal, PT_AGVGoal));

		ros::Publisher pub_handle_prohibited = nh_.advertise<geometry_msgs::Polygon>(camel_pub_topic_prohibited_area, 10);
		map_pub_handle_.insert(std::pair<std::string, ros::Publisher>(camel_pub_topic_prohibited_area, pub_handle_prohibited));
		map_pub_type_.insert(std::pair<std::string, PublishTopic>(camel_pub_topic_prohibited_area, PT_ProhibitedArea));

		ros::Publisher pub_handle_accessible = nh_.advertise<geometry_msgs::Polygon>(camel_pub_topic_accessible_area, 10);
		map_pub_handle_.insert(std::pair<std::string, ros::Publisher>(camel_pub_topic_accessible_area, pub_handle_accessible));
		map_pub_type_.insert(std::pair<std::string, PublishTopic>(camel_pub_topic_accessible_area, PT_AccessibleArea));

		ros::Publisher pub_handle_initpose = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(camel_pub_topic_initialpose, 10);
		map_pub_handle_.insert(std::pair<std::string, ros::Publisher>(camel_pub_topic_initialpose, pub_handle_initpose));
		map_pub_type_.insert(std::pair<std::string, PublishTopic>(camel_pub_topic_initialpose, PT_InitPose));

		ros::Publisher pub_handle_toPLC_notify = nh_.advertise<ros_adapter_node::camel_to_plc>(camel_pub_topic_camel2plc_notify, 1000);
		map_pub_handle_.insert(std::pair<std::string, ros::Publisher>(camel_pub_topic_camel2plc_notify, pub_handle_toPLC_notify));
		map_pub_type_.insert(std::pair<std::string, PublishTopic>(camel_pub_topic_camel2plc_notify, PT_Camel2PlcNotify));

		ros::Publisher pub_handle_CameltoPLC_notify = nh_.advertise<ros_adapter_node::camel_to_plc_direct>(camel_pub_topic_camel2plc_notify_direct, 1000);
		map_pub_handle_.insert(std::pair<std::string, ros::Publisher>(camel_pub_topic_camel2plc_notify_direct, pub_handle_CameltoPLC_notify));
		map_pub_type_.insert(std::pair<std::string, PublishTopic>(camel_pub_topic_camel2plc_notify_direct, PT_Camel2Plc));

		ros::Publisher pub_handle_toPLC_action = nh_.advertise<ros_adapter_node::camel_to_plc>(camel_pub_topic_camel2plc_action_req, 1000);
		map_pub_handle_.insert(std::pair<std::string, ros::Publisher>(camel_pub_topic_camel2plc_action_req, pub_handle_toPLC_action));
		map_pub_type_.insert(std::pair<std::string, PublishTopic>(camel_pub_topic_camel2plc_action_req, PT_Camel2PlcActionReq));

		ros::Publisher pub_handle_odom = nh_.advertise<nav_msgs::Odometry>(camel_pub_topic_odom, 1000);
		map_pub_handle_.insert(std::pair<std::string, ros::Publisher>(camel_pub_topic_odom, pub_handle_odom));
		map_pub_type_.insert(std::pair<std::string, PublishTopic>(camel_pub_topic_odom, PT_Odom));

		ros::Publisher pub_handle_toC_base = nh_.advertise<ros_adapter_node::State_msg>(camel_pub_topic_toC_base, 10);
		map_pub_handle_.insert(std::pair<std::string, ros::Publisher>(camel_pub_topic_toC_base, pub_handle_toC_base));
		map_pub_type_.insert(std::pair<std::string, PublishTopic>(camel_pub_topic_toC_base, PT_Camel2Cbase));

		ros::Publisher pub_handle_toC_points = nh_.advertise<ros_adapter_node::Point_vector_msg>(camel_pub_topic_toC_points, 10);
		map_pub_handle_.insert(std::pair<std::string, ros::Publisher>(camel_pub_topic_toC_points, pub_handle_toC_points));
		map_pub_type_.insert(std::pair<std::string, PublishTopic>(camel_pub_topic_toC_points, PT_Camel2Cpoints));

		ros::Publisher pub_handle_toC_terminal = nh_.advertise<ros_adapter_node::Terminal_msg>(camel_pub_topic_toC_terminal, 10);
		map_pub_handle_.insert(std::pair<std::string, ros::Publisher>(camel_pub_topic_toC_terminal, pub_handle_toC_terminal));
		map_pub_type_.insert(std::pair<std::string, PublishTopic>(camel_pub_topic_toC_terminal, PT_Camel2Cterminal));
		
		ros::Publisher pub_handle_toC_terminal_config = nh_.advertise<ros_adapter_node::TerminalConfig>(camel_pub_topic_toC_terminal_config, 10);
		map_pub_handle_.insert(std::pair<std::string, ros::Publisher>(camel_pub_topic_toC_terminal_config, pub_handle_toC_terminal_config));
		map_pub_type_.insert(std::pair<std::string, PublishTopic>(camel_pub_topic_toC_terminal_config, PT_Camel2CterminalConfig));

		ros::Publisher pub_handle_toPLC_stick = nh_.advertise<sensor_msgs::Joy>(camel_pub_topic_camel2plc_stick, 10);
		map_pub_handle_.insert(std::pair<std::string, ros::Publisher>(camel_pub_topic_camel2plc_stick, pub_handle_toPLC_stick));
		map_pub_type_.insert(std::pair<std::string, PublishTopic>(camel_pub_topic_camel2plc_stick, PT_Camel2PlcStick));

		ros::Publisher pub_handle_to3D_cargo_attribute = nh_.advertise<ros_adapter_node::Cargo_msg>(camel_pub_topic_to3D_cargo_attribute, 10);
		map_pub_handle_.insert(std::pair<std::string, ros::Publisher>(camel_pub_topic_to3D_cargo_attribute, pub_handle_to3D_cargo_attribute));
		map_pub_type_.insert(std::pair<std::string, PublishTopic>(camel_pub_topic_to3D_cargo_attribute, PT_Camel23DCargoAttribute));

		ros::Publisher pub_handle_to3D_activeSafe_enable = nh_.advertise<ros_adapter_node::Active_safety_enable_msg>(camel_pub_topic_to3D_is_enable, 10);
		map_pub_handle_.insert(std::pair<std::string, ros::Publisher>(camel_pub_topic_to3D_is_enable, pub_handle_to3D_activeSafe_enable));
		map_pub_type_.insert(std::pair<std::string, PublishTopic>(camel_pub_topic_to3D_is_enable, PT_Camel23DActiveSafeEnable));

		ros::Publisher pub_handle_action_list = nh_.advertise<ros_adapter_node::ActionList>(camel_pub_topic_action_list, 10);
		map_pub_handle_.insert(std::pair<std::string, ros::Publisher>(camel_pub_topic_action_list, pub_handle_action_list));
		map_pub_type_.insert(std::pair<std::string, PublishTopic>(camel_pub_topic_action_list, PT_ActionList));

		ros::Publisher pub_handle_agv_alarm = nh_.advertise<ros_adapter_node::AgvAlarmSet>(camel_pub_topic_AGV_alarm, 10);
		map_pub_handle_.insert(std::pair<std::string, ros::Publisher>(camel_pub_topic_AGV_alarm, pub_handle_agv_alarm));
		map_pub_type_.insert(std::pair<std::string, PublishTopic>(camel_pub_topic_AGV_alarm, PT_AgvAlarm));

		ros::Publisher pub_handle_agv_status = nh_.advertise<ros_adapter_node::AgvStatus>(camel_pub_topic_AGV_Status, 10);
		map_pub_handle_.insert(std::pair<std::string, ros::Publisher>(camel_pub_topic_AGV_Status, pub_handle_agv_status));
		map_pub_type_.insert(std::pair<std::string, PublishTopic>(camel_pub_topic_AGV_Status, PT_AgvStatus));

		ros::Publisher pub_handle_car_button_event = nh_.advertise<ros_adapter_node::CarButtonEvent>(camel_pub_topic_Car_Button_Event, 10);
		map_pub_handle_.insert(std::pair<std::string, ros::Publisher>(camel_pub_topic_Car_Button_Event, pub_handle_car_button_event));
		map_pub_type_.insert(std::pair<std::string, PublishTopic>(camel_pub_topic_Car_Button_Event, PT_CarButtonEvent));

		ros::Publisher pub_handle_Camel2Collaborative = nh_.advertise<ros_adapter_node::Camel2Collaborative>(camel_pub_topic_Camel2Collaborative, 1);
		map_pub_handle_.insert(std::pair<std::string, ros::Publisher>(camel_pub_topic_Camel2Collaborative, pub_handle_Camel2Collaborative));
		map_pub_type_.insert(std::pair<std::string, PublishTopic>(camel_pub_topic_Camel2Collaborative, PT_Camel2Collaborative));

		ros::Publisher pub_handle_agv_status_info = nh_.advertise<ros_adapter_node::AGVStatusInfo>(
			camel_pub_topic_agv_status_info, 10);
		map_pub_handle_.insert(std::pair<std::string, ros::Publisher>(
			camel_pub_topic_agv_status_info, pub_handle_agv_status_info));
		map_pub_type_.insert(std::pair<std::string, PublishTopic>(
			camel_pub_topic_agv_status_info, PT_AgvStatusInfo));

		// camel订阅的话题
		ros::Subscriber sub_handle_Aresult = nh_.subscribe(camel_sub_topic_localization_result,
														   10, &CRosAdapterImpl::LocalizationResultCallback, this);
		map_sub_handle_.insert(std::pair<std::string, ros::Subscriber>(camel_sub_topic_localization_result, sub_handle_Aresult));

		ros::Subscriber sub_handle_fromPLC_notify = nh_.subscribe(camel_sub_topic_plc2camel_notify,
																  1000, &CRosAdapterImpl::PLCToCamelNotifyCallback, this);
		map_sub_handle_.insert(std::pair<std::string, ros::Subscriber>(camel_sub_topic_plc2camel_notify, sub_handle_fromPLC_notify));

		ros::Subscriber sub_handle_fromPLC_direct_notify = nh_.subscribe(camel_sub_topic_plc2camel_direct,
																  1000, &CRosAdapterImpl::PLCToCamelDirectCallback, this);
		map_sub_handle_.insert(std::pair<std::string, ros::Subscriber>(camel_sub_topic_plc2camel_direct, sub_handle_fromPLC_direct_notify));

		ros::Subscriber sub_handle_fromPLC_action = nh_.subscribe(camel_sub_topic_plc2camel_action_rsp,
																  1000, &CRosAdapterImpl::PLCToCamelActionRspCallback, this);
		map_sub_handle_.insert(std::pair<std::string, ros::Subscriber>(camel_sub_topic_plc2camel_action_rsp, sub_handle_fromPLC_action));

		ros::Subscriber sub_handle_fromC_msg = nh_.subscribe(camel_sub_topic_fromC,
															 10, &CRosAdapterImpl::CToCamelMsgCallback, this);
		map_sub_handle_.insert(std::pair<std::string, ros::Subscriber>(camel_sub_topic_fromC, sub_handle_fromC_msg));

		ros::Subscriber sub_handle_QR_Check_msg = nh_.subscribe(camel_sub_topic_Check_QR_Camera,
														  10, &CRosAdapterImpl::QRCameraCheckMsgCallback, this);
		map_sub_handle_.insert(std::pair<std::string, ros::Subscriber>(camel_sub_topic_Check_QR_Camera, sub_handle_QR_Check_msg));

		ros::Subscriber sub_handle_QR_msg = nh_.subscribe(camel_sub_topic_QR_Camera,
														  10, &CRosAdapterImpl::QRCameraMsgCallback, this);
		map_sub_handle_.insert(std::pair<std::string, ros::Subscriber>(camel_sub_topic_QR_Camera, sub_handle_QR_msg));

		ros::Subscriber sub_handle_alarm_msg = nh_.subscribe(camel_sub_topic_alarm,
															 10, &CRosAdapterImpl::ROSCamelAlarmMsgCallback, this);
		map_sub_handle_.insert(std::pair<std::string, ros::Subscriber>(camel_sub_topic_alarm, sub_handle_alarm_msg));

		ros::Subscriber sub_handle_action_feedback = nh_.subscribe(camel_sub_topic_action_feedback,
																   10, &CRosAdapterImpl::ActionFeedbackCallback, this);
		map_sub_handle_.insert(std::pair<std::string, ros::Subscriber>(camel_sub_topic_action_feedback, sub_handle_action_feedback));

		ros::Subscriber sub_handle_marker_LR = nh_.subscribe(camel_sub_topic_marker_LR,
															 10, &CRosAdapterImpl::MarkerLRCallback, this);
		map_sub_handle_.insert(std::pair<std::string, ros::Subscriber>(camel_sub_topic_marker_LR, sub_handle_marker_LR));

		ros::Subscriber sub_handle_qr_error = nh_.subscribe(camel_sub_topic_QR_error_arrived, 10, &CRosAdapterImpl::QrErrorArrivedCallback, this);
		map_sub_handle_.insert(std::pair<std::string, ros::Subscriber>(camel_sub_topic_QR_error_arrived, sub_handle_qr_error));

		ros::Subscriber sub_handle_gpio = nh_.subscribe(camel_sub_topic_gpio, 10, &CRosAdapterImpl::GpioInfoCallback, this);
		map_sub_handle_.insert(std::pair<std::string, ros::Subscriber>(camel_sub_topic_gpio, sub_handle_gpio));

		ros::Subscriber sub_handle_rfid = nh_.subscribe(camel_sub_topic_rfid, 10, &CRosAdapterImpl::RfidInfoCallback, this);
		map_sub_handle_.insert(std::pair<std::string, ros::Subscriber>(camel_sub_topic_rfid, sub_handle_rfid));

		ros::Subscriber sub_handle_Collaborative2Camel = nh_.subscribe(camel_pub_topic_Collaborative2Camel, 1, &CRosAdapterImpl::Collaborative2CamelCallback, this);
		map_sub_handle_.insert(std::pair<std::string, ros::Subscriber>(camel_pub_topic_Collaborative2Camel, sub_handle_Collaborative2Camel));

		ros::Subscriber sub_handle_AgvPath = nh_.subscribe(camel_sub_topic_agv_path, 10, &CRosAdapterImpl::AgvPathCallback, this);
		map_sub_handle_.insert(std::pair<std::string, ros::Subscriber>(camel_sub_topic_agv_path, sub_handle_AgvPath));

		// camel作为client端的服务
		ros::ServiceClient terminalclient = nh_.serviceClient<ros_adapter_node::TerminalPosition>(camel_ros_service_terminal_position);
		map_service_client_.insert(std::pair<std::string, ros::ServiceClient>(camel_ros_service_terminal_position, terminalclient));
		map_service_name_.insert(std::pair<std::string, ServiceType>(camel_ros_service_terminal_position, Srv_TerminalPosition));

		ros::ServiceClient loadMapClient = nh_.serviceClient<ros_adapter_node::LoadMap>(camel_service_load_map);
		map_service_client_.insert(std::pair<std::string, ros::ServiceClient>(camel_service_load_map, loadMapClient));
		map_service_name_.insert(std::pair<std::string, ServiceType>(camel_service_load_map, Srv_LoadMap));

		ros::ServiceClient drawFlagClient = nh_.serviceClient<std_srvs::SetBool>(camel_service_draw_flag);
		map_service_client_.insert(std::pair<std::string, ros::ServiceClient>(camel_service_draw_flag, drawFlagClient));
		map_service_name_.insert(std::pair<std::string, ServiceType>(camel_service_draw_flag, Srv_DrawFlag));

		ros::ServiceClient CheckSpaceClient = nh_.serviceClient<std_srvs::SetBool>(camel_service_check_space);
		map_service_client_.insert(std::pair<std::string, ros::ServiceClient>(camel_service_check_space, CheckSpaceClient));
		map_service_name_.insert(std::pair<std::string, ServiceType>(camel_service_check_space, Srv_CheckSpace));
		

		ros::ServiceClient addRouteClient = nh_.serviceClient<ros_adapter_node::AddRoute>(camel_service_add_route);
		map_service_client_.insert(std::pair<std::string, ros::ServiceClient>(camel_service_add_route, addRouteClient));
		map_service_name_.insert(std::pair<std::string, ServiceType>(camel_service_add_route, Srv_AddRoute));

		ros::ServiceClient checkPalletDis = nh_.serviceClient<ros_adapter_node::checkPalletDis>(camel_service_check_pallet_dis);
		map_service_client_.insert(std::pair<std::string, ros::ServiceClient>(camel_service_check_pallet_dis, checkPalletDis));
		map_service_name_.insert(std::pair<std::string, ServiceType>(camel_service_check_pallet_dis, Srv_CheckPalletDis));

		ros::ServiceClient idooCheckPalletDis = nh_.serviceClient<ros_adapter_node::checkPalletDis>(camel_service_idoo_check_pallet_dis);
		map_service_client_.insert(std::pair<std::string, ros::ServiceClient>(camel_service_idoo_check_pallet_dis, idooCheckPalletDis));
		map_service_name_.insert(std::pair<std::string, ServiceType>(camel_service_idoo_check_pallet_dis, Srv_IDooCheckPalletDis));

		ros::ServiceClient checkPosision = nh_.serviceClient<ros_adapter_node::position_check>(camel_service_check_position);
		map_service_client_.insert(std::pair<std::string, ros::ServiceClient>(camel_service_check_position, checkPosision));
		map_service_name_.insert(std::pair<std::string, ServiceType>(camel_service_check_position, Srv_CheckPosision));

		ros::ServiceClient carHardwareDetails = nh_.serviceClient<ros_adapter_node::FetchCarHardwareDetails>(camel_service_car_hardware_details);
		map_service_client_.insert(std::pair<std::string, ros::ServiceClient>(camel_service_car_hardware_details, carHardwareDetails));
		map_service_name_.insert(std::pair<std::string, ServiceType>(camel_service_car_hardware_details, Srv_FetchCarHardwareDetails));
		
		ros::ServiceClient qrStationInfoClient = nh_.serviceClient<ros_adapter_node::qr_station_info>(camel_service_qr_station_info);
		map_service_client_.insert(std::pair<std::string, ros::ServiceClient>(camel_service_qr_station_info, qrStationInfoClient));
		map_service_name_.insert(std::pair<std::string, ServiceType>(camel_service_qr_station_info, Srv_QrStationInfo));
		
		// camel作为server端的服务
		ros::ServiceServer avoid_service = nh_.advertiseService(camel_ros_service_avoid_operation,
																&CRosAdapterImpl::AvoidOperationRequestCallback, this);
		map_service_server_.insert(std::pair<std::string, ros::ServiceServer>(camel_ros_service_avoid_operation, avoid_service));

		ros::ServiceServer alarm_info_service = nh_.advertiseService(camel_service_alarm_info, &CRosAdapterImpl::AlarmInfoServiceCallback, this);
		map_service_server_.insert(std::pair<std::string, ros::ServiceServer>(camel_service_alarm_info, alarm_info_service));

		ros::ServiceServer running_info_service = nh_.advertiseService(camel_service_running_info, &CRosAdapterImpl::RunningInfoServiceCallback, this);
		map_service_server_.insert(std::pair<std::string, ros::ServiceServer>(camel_service_running_info, running_info_service));

		ros::ServiceServer position_info_service = nh_.advertiseService(camel_service_position_info, &CRosAdapterImpl::PositionInfoServiceCallback, this);
		map_service_server_.insert(std::pair<std::string, ros::ServiceServer>(camel_service_position_info, position_info_service));

		ros::ServiceServer system_info_service = nh_.advertiseService(camel_service_system_info, &CRosAdapterImpl::SystemInfoServiceCallback, this);
		map_service_server_.insert(std::pair<std::string, ros::ServiceServer>(camel_service_system_info, system_info_service));

		ros::ServiceServer task_info_service = nh_.advertiseService(camel_service_task_info, &CRosAdapterImpl::TaskInfoServiceCallback, this);
		map_service_server_.insert(std::pair<std::string, ros::ServiceServer>(camel_service_task_info, task_info_service));

		spnr_.start();
	}

	CRosAdapterImpl* CRosAdapterImpl::GetInstanse()
	{
		static CRosAdapterImpl * impl = nullptr;

		if (!impl)
		{
			int dummy_argc = 0;
    	 	char** dummy_argv = nullptr;
			std::cout << "init ros node, please wait..." << std::endl;
			ros::init(dummy_argc, dummy_argv, "agv_camel_service", ros::init_options::NoRosout);
			ros::NodeHandle n;
			impl = new CRosAdapterImpl();
			impl->Init(n);
		}

		return impl;
	}

	void CRosAdapterImpl::Release()
	{
		ROS_INFO("enter rosadapter release!");
		spnr_.stop();
		psubhandle = nullptr;
		ros::shutdown();
	}

	bool CRosAdapterImpl::CallService(const std::string &service, void *prequest, void *presponse)
	{
		if (map_service_client_.count(service) == 0)
		{
			return false;
		}

		ros::ServiceClient client = map_service_client_[service];

		ServiceType srv_type = map_service_name_[service];
		switch (srv_type)
		{
		case Srv_TerminalPosition:
		{
			TerminalRequest *pterminal_request = (TerminalRequest *)prequest;
			TerminalResponse *pterminal_response = (TerminalResponse *)presponse;
			ros_adapter_node::TerminalPosition srvmsg;
			EncodeRosTerminalMsg(pterminal_request, srvmsg);
			if (client.call(srvmsg))
			{
				// encode response message
				pterminal_response->error_code = srvmsg.response.errorcode;
				pterminal_response->error_msg = srvmsg.response.errormsg;
				return true;
			}
			else
			{
				ROS_ERROR("camel ros node call terminal service failed");
				// log error
				return false;
			}
		}
		case Srv_LoadMap:
		{
			if (prequest == nullptr)
				return false;
			LoadMapRequest *request = (LoadMapRequest *)prequest;
			ServiceResponse *response = (ServiceResponse *)presponse;

			ros_adapter_node::LoadMap srvmsg;
			srvmsg.request.map_name = request->map_name;
			srvmsg.request.initial_pose = request->initial_pose;
			srvmsg.request.x = request->x;
			srvmsg.request.y = request->y;
			srvmsg.request.yaw = request->yaw;
			if (client.call(srvmsg))
			{
				// encode response message
				response->success = srvmsg.response.success;
				response->err_msg = srvmsg.response.err_msg;
				return true;
			}
			else
			{
				ROS_ERROR("camel ros node call load map service failed");
				// log error
				return false;
			}
		}
		case Srv_DrawFlag:
		{
			if (prequest == nullptr)
				return false;
			DrawFlagRequest *request = (DrawFlagRequest *)prequest;
			ServiceResponse *response = (ServiceResponse *)presponse;

			std_srvs::SetBool srvmsg;
			srvmsg.request.data = request->flag;
			if (client.call(srvmsg))
			{
				response->success = srvmsg.response.success;
				response->err_msg = srvmsg.response.message;
				return true;
			}
			else
			{
				ROS_ERROR("camel ros node call request draw path service failed");
				// log error
				return false;
			}
		}
		case Srv_CheckSpace:
		{
			if (prequest == nullptr)
				return false;
			CheckSpaceRequest *request = (CheckSpaceRequest *)prequest;
			ServiceResponse *response = (ServiceResponse *)presponse;

			std_srvs::SetBool srvmsg;
			srvmsg.request.data = request->flag;
			if (client.call(srvmsg))
			{
				response->success = srvmsg.response.success;
				response->err_msg = srvmsg.response.message;
				return true;
			}
			else
			{
				ROS_ERROR("camel ros node call request draw path service failed");
				// log error
				return false;
			}
		}
		case Srv_AddRoute:
		{
			if (nullptr == prequest)
				return false;
			AGVPaths *request = (AGVPaths *)prequest;
			AddRouteResponse *response = (AddRouteResponse *)presponse;

			ros_adapter_node::AddRoute srvmsg;
			for (auto &path : request->paths)
			{
				ros_adapter_node::path oldPath;
				oldPath.start_pt.x = path.start_pose.x;
				oldPath.start_pt.y = path.start_pose.y;
				oldPath.start_pt.theta = path.start_pose.theta;
				oldPath.end_pt.x = path.end_pose.x;
				oldPath.end_pt.y = path.end_pose.y;
				oldPath.end_pt.theta = path.end_pose.theta;
				oldPath.radius = path.radius;
				oldPath.direct = path.direct;
				oldPath.start_id = path.start_id;
				oldPath.startIsTaskPoint = path.startIsTaskPoint;
				oldPath.end_id = path.end_id;
				oldPath.endIsTaskPoint = path.endIsTaskPoint;

				srvmsg.request.paths.paths.push_back(oldPath);
			}

			if (client.call(srvmsg))
			{
				response->err_msgs = srvmsg.response.err_msgs;
				response->success = srvmsg.response.success;
				for (auto &path : srvmsg.response.paths.paths)
				{
					Path newPath;
					newPath.start_pose.x = path.start_pt.x;
					newPath.start_pose.y = path.start_pt.y;
					newPath.start_pose.theta = path.start_pt.theta;
					newPath.end_pose.x = path.end_pt.x;
					newPath.end_pose.y = path.end_pt.y;
					newPath.end_pose.theta = path.end_pt.theta;
					newPath.radius = path.radius;
					newPath.direct = path.direct;
					newPath.start_id = path.start_id;
					newPath.startIsTaskPoint = path.startIsTaskPoint;
					newPath.end_id = path.end_id;
					newPath.endIsTaskPoint = path.endIsTaskPoint;

					response->paths.paths.push_back(newPath);
				}
				return true;
			}
			else
			{
				ROS_ERROR("camel ros node call request add route service failed");
				return false;
			}
		}
		case Srv_CheckPalletDis:
		case Srv_IDooCheckPalletDis:
		{
			ros_adapter_node::checkPalletDis srvmsg;
			if (client.call(srvmsg))
			{
				((CheckPalletDis *)presponse)->distance = srvmsg.response.distance;
				return true;
			}
			return false;
		}
		case Srv_CheckPosision :
		{
			if (nullptr == prequest)
				return false;
			float *request = (float *)prequest;
			ros_adapter_node::position_check msg;
			msg.request.err_threshold = *request;
			bool *rsp = (bool *)presponse;
			if (client.call(msg))
			{
				*rsp = msg.response.response_value;
				return true;
			}
			break;
		}
		case Srv_FetchCarHardwareDetails:
		{
			ros_adapter_node::FetchCarHardwareDetails msg;
			CarHardwareDetails *rsp = (CarHardwareDetails *)presponse;
			if (client.call(msg))
			{
				rsp->leftOpticalSignal = msg.response.leftOpticalSignal;
				rsp->rightOpticalSignal = msg.response.rightOpticalSignal;
				rsp->forkGear = msg.response.forkGear;
				return true;
			}
			break;
		}
		case Srv_QrStationInfo:
		{
			if (prequest == nullptr)
				return false;
			QrStationInfoRequest *request = (QrStationInfoRequest *)prequest;
			ServiceResponse *response = (ServiceResponse *)presponse;

			ros_adapter_node::qr_station_info srvmsg;
			srvmsg.request.station_and_qr_info = request->station_and_qr_info;
			
			if (client.call(srvmsg))
			{
				response->success = srvmsg.response.success;
				return true;
			}
			else
			{
				ROS_ERROR("camel ros node call request station list service failed");
				// log error
				return false;
			}
			break;
		}
		default:
			break;
		}
		return false;
	}

	void CRosAdapterImpl::EncodeRosAgvGoalMsg(AGVGoal *&pmsg, ros_adapter_node::Navigation &rosmsg)
	{
		if (nullptr == pmsg)
		{
			return;
		}
		rosmsg.header.seq = CRosAdapterImpl::ros_req_id;
		CRosAdapterImpl::ros_req_id++;
		rosmsg.header.stamp = ros::Time::now();
		rosmsg.header.frame_id = "map";
		rosmsg.pose.position.x = pmsg->pose.x;
		rosmsg.pose.position.y = pmsg->pose.y;
		rosmsg.pose.position.z = pmsg->pose.z;

		rosmsg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pmsg->pose.theta);
		rosmsg.v = pmsg->v;
		rosmsg.w = pmsg->w;
	}

	void CRosAdapterImpl::EncodeRosInitPoseMsg(CarPose *&pmsg, geometry_msgs::PoseWithCovarianceStamped &rosmsg)
	{
		if (nullptr == pmsg)
		{
			return;
		}
		rosmsg.header.seq = CRosAdapterImpl::ros_req_id;
		CRosAdapterImpl::ros_req_id++;
		rosmsg.header.stamp = ros::Time::now();
		rosmsg.header.frame_id = "map";
		rosmsg.pose.pose.position.x = pmsg->x;
		rosmsg.pose.pose.position.y = pmsg->y;
		rosmsg.pose.pose.position.z = pmsg->z;

		rosmsg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pmsg->theta);
	}

	void CRosAdapterImpl::EncodeRosTerminalMsg(TerminalRequest *&pmsg, ros_adapter_node::TerminalPosition &rosmsg)
	{
		if (nullptr == pmsg)
		{
			return;
		}
		rosmsg.request.flag = pmsg->flag;
		rosmsg.request.pose.position.x = pmsg->pose.x;
		rosmsg.request.pose.position.y = pmsg->pose.y;
		rosmsg.request.pose.position.z = pmsg->pose.z;
		rosmsg.request.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pmsg->pose.theta);
		rosmsg.request.target_type = pmsg->target_type;
	}

	double CRosAdapterImpl::QuaternionToRPY(const geometry_msgs::Quaternion &quaternion)
	{
		tf::Quaternion quat(quaternion.x, quaternion.y, quaternion.z, quaternion.w);

		double roll = 0;
		double pitch = 0;
		double yaw = 0;
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); // 进行转换
		return yaw;
	}

	void CRosAdapterImpl::EncodeRosOdomMsg(CamelOdom *&pmsg, nav_msgs::Odometry &rosmsg)
	{
		if (nullptr == pmsg)
		{
			return;
		}
		rosmsg.header.seq = CRosAdapterImpl::ros_req_id;
		CRosAdapterImpl::ros_req_id++;
		rosmsg.header.stamp = ros::Time::now();
		rosmsg.header.frame_id = "odom";
		rosmsg.pose.pose.position.x = pmsg->pose.x;
		rosmsg.pose.pose.position.y = pmsg->pose.y;
		rosmsg.pose.pose.position.z = 0;

		rosmsg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pmsg->pose.theta);

		rosmsg.child_frame_id = "base_link";
		rosmsg.twist.twist.linear.x = pmsg->vx;
		rosmsg.twist.twist.linear.y = pmsg->vy;
		rosmsg.twist.twist.angular.z = pmsg->w;
	}

	void CRosAdapterImpl::EncodeRosCamel2CBaseMsg(CINBASE *&pmsg, ros_adapter_node::State_msg &rosmsg)
	{
		if (nullptr == pmsg)
		{
			return;
		}
		rosmsg.id = pmsg->id;
		rosmsg.enable = pmsg->enable;
		rosmsg.status = pmsg->status;
		rosmsg.mode = pmsg->mode;
		rosmsg.dataClear = pmsg->dataClear;
		rosmsg.rotate_type = pmsg->rotate.rotate_type;
		rosmsg.rotate_goal = pmsg->rotate.rotate_goal;
		rosmsg.cargo = pmsg->cargo;
		rosmsg.control_mode = pmsg->controlMode;
	}

	void CRosAdapterImpl::EncodeRosCamel2CPointsMsg(std::vector<CPOINT> *&pmsg, ros_adapter_node::Point_vector_msg &rosmsg)
	{
		if (nullptr == pmsg)
		{
			return;
		}
		for (auto point : (*pmsg))
		{
			ros_adapter_node::Point_msg rosPoint;
			rosPoint.point_x = point.x;
			rosPoint.point_y = point.y;
			rosPoint.point_theta = point.theta;
			rosPoint.point_velocity = point.velocity;
			rosPoint.point_radius = point.radius;
			rosPoint.destype = point.destype;
			rosPoint.controlMethod = point.controlMethod;
			// rosPoint.QR_id    		= point.QR_id;

			rosmsg.point.push_back(rosPoint);
		}
	}

	void CRosAdapterImpl::EncodeRosCamel2CTerminalMsg(RELATIVE_POS *&pmsg, ros_adapter_node::Terminal_msg &rosmsg)
	{
		if (nullptr == pmsg)
		{
			return;
		}
		rosmsg.terminal_enable = pmsg->terminal_enable;
		rosmsg.isdatavalid = pmsg->isdatavalid;
		rosmsg.deltax = pmsg->deltax;
		rosmsg.deltay = pmsg->deltay;
		rosmsg.deltatheta = pmsg->deltatheta;
		rosmsg.distance_maxerror = pmsg->distance_maxerror;
		rosmsg.angle_maxerror = pmsg->angle_maxerror;
		rosmsg.terminal_type = pmsg->terminal_type;
		rosmsg.check_point = pmsg->check_point;
		rosmsg.expect_ID = pmsg->expect_ID;
		rosmsg.real_ID = pmsg->real_ID;
		rosmsg.deltay = pmsg->deltay;
	}
	
	void CRosAdapterImpl::EncodeRosCamel2CTerminalConfig(TERMINAL_CONFIG *&pmsg, ros_adapter_node::TerminalConfig &rosmsg)
	{
		if (nullptr == pmsg)
		{
			return;
		}
		rosmsg.target_type = pmsg->target_type;
		rosmsg.deltax = pmsg->deltax;
		rosmsg.deltay = pmsg->deltay;
		rosmsg.deltatheta = pmsg->deltatheta;
	}

	void CRosAdapterImpl::EncodeActionList(ActionList *pmsg, ros_adapter_node::ActionList &rosmsg)
	{
		if (nullptr == pmsg)
		{
			return;
		}
		rosmsg.header.seq = CRosAdapterImpl::ros_req_id++;
		rosmsg.header.stamp = ros::Time::now();
		rosmsg.header.frame_id = "camel";
		for (const auto &action : pmsg->action_list)
		{
			ros_adapter_node::ActionEntry rosaction;

			rosaction.action_type = (uint8_t)action.action_type;
			rosaction.ns = action.ns;
			rosaction.action_id = action.action_id;
			for (const auto &pair : action.params)
			{
				ros_adapter_node::KeyValue keyValue;
				keyValue.key = pair.first;
				keyValue.value = pair.second;
				rosaction.params.push_back(std::move(keyValue));
			}
			rosmsg.action_list.push_back(std::move(rosaction));
		}
	}

	void CRosAdapterImpl::EncodeAgvAlarm(std::vector<AgvAlarm> *&pmsg, ros_adapter_node::AgvAlarmSet &rosmsg)
	{
		if (nullptr == pmsg)
		{
			return;
		}
		rosmsg.header.seq = CRosAdapterImpl::ros_req_id++;
		rosmsg.header.stamp = ros::Time::now();
		rosmsg.header.frame_id = "AgvAlarm";
		for (auto alarm : (*pmsg))
		{
			ros_adapter_node::AgvAlarmEntry info;
			info.id = alarm.id;
			info.source = alarm.source;
			info.level = alarm.level;
			info.status = alarm.status;
			info.message = alarm.message;

			rosmsg.alarmInfo.push_back(info);
		}
	}

	void CRosAdapterImpl::EncodeAgvStatus(CamelCarState *&pmsg, ros_adapter_node::AgvStatus &rosmsg)
	{
		if (nullptr == pmsg)
		{
			return;
		}
		rosmsg.carName = pmsg->carName;
		rosmsg.currentStation = pmsg->currentStation;
		rosmsg.batteryPercent = pmsg->batteryPercent;
		rosmsg.chargingState = pmsg->chargingState;
		rosmsg.ctrlMode = pmsg->ctrlMode;
		rosmsg.ctrlStatus = pmsg->ctrlStatus;
		rosmsg.hasRoute = pmsg->hasRoute;
		rosmsg.cargoState = pmsg->cargoState;
		rosmsg.mapNotMatch = pmsg->mapNotMatch;
		rosmsg.onRouteEnd = pmsg->onRouteEnd;
	}

	void CRosAdapterImpl::EncodeCarButtonEvent(CarButtonEvent *pmsg, ros_adapter_node::CarButtonEvent &rosmsg)
	{
		if (nullptr == pmsg)
			return;

		rosmsg.on_reset_press = pmsg->onResetPress;
		rosmsg.on_reset_release = pmsg->onResetRelease;
		rosmsg.on_reset_long_press = pmsg->onResetLongPress;
	}

	void CRosAdapterImpl::EncodeCamel2Collaborative(Camel2Collaborative *pmsg, ros_adapter_node::Camel2Collaborative &rosmsg)
	{
		if (nullptr == pmsg)
			return;

		rosmsg.Start_cooperation = pmsg->startCooperation;
		rosmsg.HeadCar_ip = pmsg->headCarIp;
		rosmsg.RearCar_ip = pmsg->rearCarIp;
		rosmsg.Reach_mission = pmsg->reach_mission;
		rosmsg.End_cooperation = pmsg->endCooperation;
		rosmsg.MasterSlave_identity = pmsg->masterSlaveIdentity;
	}

	void CRosAdapterImpl::EncodeAgvStatusInfo(AGVStatusInfo* pmsg, ros_adapter_node::AGVStatusInfo& rosmsg)
	{
		if (nullptr == pmsg)
		{
			return;
		}

		rosmsg.header.seq = CRosAdapterImpl::ros_req_id++;
		rosmsg.header.stamp = ros::Time::now();
		rosmsg.header.frame_id = "agv_status";

		rosmsg.current_station = pmsg->current_station;
		rosmsg.station_type = pmsg->station_type;
		rosmsg.distance_to_target = pmsg->distance_to_target;
		rosmsg.current_x = pmsg->current_x;
		rosmsg.current_y = pmsg->current_y;
		rosmsg.current_theta = pmsg->current_theta;
		rosmsg.has_route = pmsg->has_route;
		rosmsg.has_task = pmsg->has_task;
	}

	void CRosAdapterImpl::LocalizationResultCallback(const ros_adapter_node::LocalizationResult::ConstPtr &pmsg)
	{
		if ((pmsg == nullptr) || (psubhandle == nullptr))
		{
			return;
		}

		ResultFromA resultmsg;
		resultmsg.stamp_secs = pmsg->time_stamp.sec;
		resultmsg.stamp_nsecs = pmsg->time_stamp.nsec;
		resultmsg.seq_id = pmsg->seq_id;
		resultmsg.loc_state = pmsg->loc_state;
		resultmsg.certainty = pmsg->certainty;
		resultmsg.pose.x = pmsg->pose_with_covariance.pose.position.x;
		resultmsg.pose.y = pmsg->pose_with_covariance.pose.position.y;
		resultmsg.pose.z = pmsg->pose_with_covariance.pose.position.z;
		resultmsg.pose.theta = QuaternionToRPY(pmsg->pose_with_covariance.pose.orientation);
		resultmsg.frame = pmsg->frame;
		resultmsg.ref_frame = pmsg->ref_frame;

		psubhandle->LocalizationResultHandle(resultmsg);
	}

	void CRosAdapterImpl::PLCToCamelNotifyCallback(const ros_adapter_node::plc_to_camel::ConstPtr &pmsg)
	{
		if ((pmsg == nullptr) || (psubhandle == nullptr))
		{
			return;
		}

		char datainfo[camel_plc_msg_max_length];
		memcpy(datainfo, pmsg->plc_to_camel_message.data(), pmsg->length);
		psubhandle->OnGetRosPlcCycleNotify(datainfo, pmsg->length);
	}

	void CRosAdapterImpl::PLCToCamelDirectCallback(const ros_adapter_node::plc_to_camel_direct::ConstPtr &pmsg)
	{
		if ((pmsg == nullptr) || (psubhandle == nullptr))
		{
			return;
		}
		CamelGetPLCData data;
		data.InManualCharging = pmsg->InManualCharging;
		data.SemiAutomaticMode = pmsg->SemiAutomaticMode;

		psubhandle->OnGetRosPlcDirectdata(data);
	}

	void CRosAdapterImpl::PLCToCamelActionRspCallback(const ros_adapter_node::plc_to_camel::ConstPtr &pmsg)
	{
		if ((pmsg == nullptr) || (psubhandle == nullptr))
		{
			return;
		}
		char datainfo[camel_plc_msg_max_length];
		memcpy(datainfo, pmsg->plc_to_camel_message.data(), pmsg->length);
		psubhandle->OnGetRosPlcActionRsp(datainfo, pmsg->length);
	}

	void CRosAdapterImpl::CToCamelMsgCallback(const ros_adapter_node::Controller2Camel_msg::ConstPtr &pmsg)
	{
		if ((pmsg == nullptr) || (psubhandle == nullptr))
		{
			return;
		}

		C2CAMEL fromC;
		fromC.id = pmsg->id;
		fromC.Vx = pmsg->Vx;
		fromC.Vy = pmsg->Vy;
		fromC.w = pmsg->w;
		fromC.status = pmsg->status;
		fromC.od1 = pmsg->od1;
		fromC.od2 = pmsg->od2;
		fromC.indexFromEnd = pmsg->indexFromEnd;
		fromC.dataClearDone = pmsg->dataClearDone;
		fromC.autoObsAvoid = pmsg->autoObsAvoid;
		fromC.cir_Flag = pmsg->cir_Flag;
		fromC.coord.x = pmsg->coord.x;
		fromC.coord.y = pmsg->coord.y;
		fromC.coord.theta = pmsg->coord.theta;
		fromC.terminal_status = pmsg->terminal_status;
		fromC.rotate_finished = pmsg->rotate_finished;
		fromC.typevw = pmsg->typevw;
		memcpy(fromC.alarmCode, pmsg->alarmCode.begin(), sizeof(fromC.alarmCode));

		psubhandle->OnGetC2CamelMsg(&fromC);
	}

	bool CRosAdapterImpl::AvoidOperationRequestCallback(ros_adapter_node::AvoidOperation::Request &req, ros_adapter_node::AvoidOperation::Response &rsp)
	{
		if (psubhandle == nullptr)
		{
			return false;
		}
		psubhandle->OnReceiveAvoidOperationRequest(req.operation, rsp.errorcode);
		return true;
	}

	bool CRosAdapterImpl::GetCarStateCallback(ros_adapter_node::CarState::Request &req, ros_adapter_node::CarState::Response &rsp)
	{
		if (psubhandle == nullptr)
		{
			return false;
		}
		CamelCarState state;
		psubhandle->OnGetCarState(state);

		/*rsp.batteryPercent = state.batteryPercent;
		rsp.chargingState  = state.chargingState;
		rsp.cargoState 	  = state.cargoState;
		rsp.frontDetect   = state.frontDetect;
		rsp.backDetect 	  = state.backDetect;
		rsp.inDetect 	  = state.inDetect;
		rsp.leftFork 	  = state.leftFork;
		rsp.rightFork 	  = state.rightFork;
		rsp.redLight 	  = state.redLight;
		rsp.ctrlMode 	  = state.ctrlMode;
		rsp.currentStation = state.currentStation;
		rsp.carName   	  = state.carName;*/

		return true;
	}

	void CRosAdapterImpl::QRCameraCheckMsgCallback(const ros_adapter_node::QRInfo::ConstPtr &pmsg)
	{
		if ((pmsg == nullptr) || (psubhandle == nullptr))
		{
			return;
		}

		CamelQRInfo qr_info;
		qr_info.id = pmsg->ID;
		qr_info.offset_x = pmsg->Offset_X;
		qr_info.offset_y = pmsg->Offset_Y;
		qr_info.angle = pmsg->Angle;
		psubhandle->OnGetQRCameraCheckMsg(qr_info);
	}

	void CRosAdapterImpl::QRCameraMsgCallback(const ros_adapter_node::QRInfo::ConstPtr &pmsg)
	{
		if ((pmsg == nullptr) || (psubhandle == nullptr))
		{
			return;
		}

		CamelQRInfo qr_info;
		qr_info.id = pmsg->ID;
		qr_info.offset_x = pmsg->Offset_X;
		qr_info.offset_y = pmsg->Offset_Y;
		qr_info.angle = pmsg->Angle;

		psubhandle->OnGetQRCameraMsg(qr_info);
	}

	void CRosAdapterImpl::ROSCamelAlarmMsgCallback(const ros_adapter_node::ErrorSet::ConstPtr &pmsg)
	{
		if ((pmsg == nullptr) || (psubhandle == nullptr))
		{
			return;
		}

		CamelROSAlarmInfo alarmInfo;
		alarmInfo.source = pmsg->source;
		for (auto alarm : pmsg->err_codes)
		{
			CamelROSErrorCodesInfo errorInfo;
			errorInfo.code_id = alarm.code_id;
			errorInfo.params = alarm.params;
			alarmInfo.err_codes.push_back(errorInfo);
		}
		psubhandle->OnGetAlarmsMsg(alarmInfo);
	}

	void CRosAdapterImpl::ActionFeedbackCallback(const ros_adapter_node::ActionFeedback::ConstPtr &pmsg)
	{
		if ((pmsg == nullptr) || (psubhandle == nullptr))
		{
			return;
		}

		ActionFeedback feedback;
		feedback.header.stamp_secs = pmsg->header.stamp.sec;
		feedback.header.stamp_nsecs = pmsg->header.stamp.nsec;
		feedback.header.seq_id = pmsg->header.seq;
		feedback.header.frame = pmsg->header.frame_id;
		feedback.result = (ActionFeedback::ActionResult)pmsg->result;
		feedback.action_id = pmsg->action_id;
		feedback.err_msg = pmsg->err_msg;
		psubhandle->OnGetActionFeedback(feedback);
	}

	void CRosAdapterImpl::MarkerLRCallback(const ros_adapter_node::LocalizationResult::ConstPtr &pmsg)
	{
		if ((pmsg == nullptr) || (psubhandle == nullptr))
		{
			return;
		}

		ResultFromA resultmsg;
		resultmsg.stamp_secs = pmsg->time_stamp.sec;
		resultmsg.stamp_nsecs = pmsg->time_stamp.nsec;
		resultmsg.seq_id = pmsg->seq_id;
		resultmsg.loc_state = pmsg->loc_state;
		resultmsg.certainty = pmsg->certainty;
		resultmsg.pose.x = pmsg->pose_with_covariance.pose.position.x;
		resultmsg.pose.y = pmsg->pose_with_covariance.pose.position.y;
		resultmsg.pose.z = pmsg->pose_with_covariance.pose.position.z;
		resultmsg.pose.theta = QuaternionToRPY(pmsg->pose_with_covariance.pose.orientation);
		resultmsg.frame = pmsg->frame;
		resultmsg.ref_frame = pmsg->ref_frame;

		psubhandle->OnGetMarkerLR(resultmsg);
	}

	void CRosAdapterImpl::QrErrorArrivedCallback(const ros_adapter_node::QRerrorArrived_msg::ConstPtr &pmsg)
	{
		if ((pmsg == nullptr) || (psubhandle == nullptr))
		{
			return;
		}
		QrErrorArrived error;
		error.arrive = pmsg->arrive;
		error.dataValid = pmsg->ArriveDataValid;
		error.x = pmsg->ArriveDatax;
		error.y = pmsg->ArriveDatay;
		error.theta = pmsg->ArriveDatatheta;
		psubhandle->OnGetQrErrorArrived(error);
	}

	void CRosAdapterImpl::Collaborative2CamelCallback(const ros_adapter_node::Collaborative2Camel::ConstPtr &pmsg)
	{
		if ((pmsg == nullptr) || (psubhandle == nullptr))
		{
			return;
		}

		Collaborative2Camel msg;
		msg.colla_x = pmsg->colla_x;
		msg.colla_y = pmsg->colla_y;
		msg.colla_theta = pmsg->colla_theta;
		msg.cooperationStatus = pmsg->Cooperation_status;
		psubhandle->OnCollaborative2Camel(msg);
	}

	void CRosAdapterImpl::RfidInfoCallback(const ros_adapter_node::rfid::ConstPtr &pmsg)
	{
		if ((pmsg == nullptr) || (psubhandle == nullptr))
		{
			return;
		}
		RfidInfo info;
		info.rfid = pmsg->RFID;
		psubhandle->OnGetRfidInfo(info);
	}

	void CRosAdapterImpl::GpioInfoCallback(const ros_adapter_node::gpio::ConstPtr &pmsg)
	{
		if ((pmsg == nullptr) || (psubhandle == nullptr))
		{
			return;
		}
		GpioInfo info;
		info.magnetic_front = pmsg->magnetic_front;
		info.magnetic_back = pmsg->magnetic_back;
		info.magnetic_break = pmsg->magnetic_break;
		psubhandle->OnGetGpioInfo(info);
	}

	bool CRosAdapterImpl::AlarmInfoServiceCallback(ros_adapter_node::AlarmInfo::Request &req, ros_adapter_node::AlarmInfo::Response &rsp)
	{
		if (psubhandle == nullptr)
		{
			return false;
		}
		std::vector<AgvAlarm> alarmInfo;
		psubhandle->GetAlarmInfo(alarmInfo);

		for (auto it : alarmInfo)
		{
			ros_adapter_node::AgvAlarmEntry info;
			info.id = it.id;
			info.source = it.source;
			info.level = it.level;
			info.status = it.status;
			info.message = it.message;
			rsp.alarmInfo.push_back(info);
		}

		return true;
	}

	bool CRosAdapterImpl::PositionInfoServiceCallback(ros_adapter_node::PositionInfo::Request &req, ros_adapter_node::PositionInfo::Response &rsp)
	{
		if (psubhandle == nullptr)
		{
			return false;
		}
		AGVPositionInfo info;
		psubhandle->GetPositionInfo(info);
		rsp.currentStation = info.currentStation;
		rsp.x = info.x;
		rsp.y = info.y;
		rsp.theta = info.theta;
		rsp.certainty = info.certainty;

		return true;
	}

	bool CRosAdapterImpl::RunningInfoServiceCallback(ros_adapter_node::RunningInfo::Request &req, ros_adapter_node::RunningInfo::Response &rsp)
	{
		if (psubhandle == nullptr)
		{
			return false;
		}

		AGVRunningInfo info;
		psubhandle->GetRunningInfo(info);
		rsp.mode = info.mode;
		rsp.status = info.status;
		rsp.vxReal = info.vxReal;
		rsp.vyReal = info.vyReal;
		rsp.wReal = info.wReal;
		rsp.vxExp = info.vxExp;
		rsp.vyExp = info.vyExp;
		rsp.wExp = info.wExp;
		rsp.battery = info.battery;
		rsp.chargeState = info.chargeState;
		rsp.cargo = info.cargo;
		return true;
	}

	bool CRosAdapterImpl::SystemInfoServiceCallback(ros_adapter_node::SystemInfo::Request &req, ros_adapter_node::SystemInfo::Response &rsp)
	{
		if (psubhandle == nullptr)
		{
			return false;
		}

		AGVSystemInfo info;
		psubhandle->GetSystemInfo(info);
		rsp.carName = info.carName;
		rsp.version = info.version;
		rsp.mapInfo = info.mapInfo;
		rsp.ipAddr = info.ipAddr;
		rsp.mac = info.mac;
		rsp.cpuUsage = info.cpuUsage;
		rsp.memUsage = info.memUsage;
		return true;
	}

	bool CRosAdapterImpl::TaskInfoServiceCallback(ros_adapter_node::TaskInfo::Request &req, ros_adapter_node::TaskInfo::Response &rsp)
	{
		if (psubhandle == nullptr)
		{
			return false;
		}

		AGVTaskInfo info;
		psubhandle->GetTaskInfo(info);
		rsp.route = info.route;
		rsp.targetStation = info.targetStation;
		return true;
	}

	void CRosAdapterImpl::AgvPathCallback(const ros_adapter_node::Paths::ConstPtr &pmsg)
	{
		if ((pmsg == nullptr) || (psubhandle == nullptr))
		{
			return;
		}

		AGVPaths agvPaths;
		for (auto it : pmsg->paths)
		{
			Path path;
			path.direct = it.direct;
			path.start_pose.x = it.start_pt.x;
			path.start_pose.y = it.start_pt.y;
			path.start_pose.theta = it.start_pt.theta;
			path.end_pose.x = it.end_pt.x;
			path.end_pose.y = it.end_pt.y;
			path.end_pose.theta = it.end_pt.theta;
			path.radius = it.radius;
			path.start_id = it.start_id;
			path.startIsTaskPoint = it.startIsTaskPoint;
			path.end_id = it.end_id;
			path.endIsTaskPoint = it.endIsTaskPoint;

			agvPaths.paths.push_back(path);
		}
		psubhandle->OnGetAgvPath(agvPaths);
	}

	std::atomic<unsigned int> CRosAdapterImpl::ros_req_id = {0};

	CRosPublish *CRosFactory::CreatePublish()
	{
		return CRosAdapterImpl::GetInstanse();
		// ros节点初始化需要点时间，如果初始化后立刻调用publish接口，可能发不出消息
		// ros::Duration(2.0).sleep();
	}

	void CRosFactory::DeletePublish(CRosPublish *pPublish)
	{
		/*
		CRosAdapterImpl *p = (CRosAdapterImpl *)pPublish;
		if (p != nullptr)
		{
			delete p;
		}
		*/
	}

    bool CRosAdapterImpl::HasConfig(const std::string &key)
    {
        return nh_.hasParam(key);
    }

    bool CRosAdapterImpl::GetConfig(const std::string &key, std::string &value)
    {
        return nh_.getParam(key, value);
    }
    bool CRosAdapterImpl::GetConfig(const std::string& key, int& value) {
		return nh_.getParam(key, value);
	}
	bool CRosAdapterImpl::GetConfig(const std::string& key, bool& value) {
		return nh_.getParam(key, value);
	}
	bool CRosAdapterImpl::GetConfig(const std::string& key, float& value) {
		return nh_.getParam(key, value);
	}

	bool CRosAdapterImpl::SetConfig(const std::string& key, const std::string& value) {
    	nh_.setParam(key, value);
		return true;
	}
	bool CRosAdapterImpl::SetConfig(const std::string& key, const int& value) 
	{
		nh_.setParam(key, value);
		return true;
	}

	bool CRosAdapterImpl::SetConfig(const std::string& key, const bool& value) {
    	nh_.setParam(key, value);
		return true;
    }
	bool CRosAdapterImpl::SetConfig(const std::string& key, const float& value) {
    	nh_.setParam(key, value);
		return true;
	}

	bool CRosAdapterImpl::GetConfig(const std::string &key, std::map<std::string, bool> &maps)
	{
		return nh_.getParam(key, maps);
	}

	CRosConfigHandle* CRosFactory::GetRosConfig() {
		return CRosAdapterImpl::GetInstanse();
	}
}
