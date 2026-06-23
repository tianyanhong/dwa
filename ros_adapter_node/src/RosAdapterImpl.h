#include <map>
#include <memory>
#include <atomic>
#include <sensor_msgs/Joy.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "ros_adapter_node/Navigation.h"
#include "ros_adapter_node/LocalizationResult.h"
#include "ros_adapter_node/AGVStatusInfo.h"
#include "ros_adapter_node/camel_to_plc.h"
#include "ros_adapter_node/camel_to_plc_direct.h"
#include "ros_adapter_node/plc_to_camel_direct.h"
#include "ros_adapter_node/camel_to_plc_direct.h"
#include "ros_adapter_node/plc_to_camel.h"
#include "ros_adapter_node/TerminalPosition.h"
#include "ros_adapter_node/Controller2Camel_msg.h"
#include "ros_adapter_node/Coord_msg.h"
#include "ros_adapter_node/Point_msg.h"
#include "ros_adapter_node/Point_vector_msg.h"
#include "ros_adapter_node/State_msg.h"
#include "ros_adapter_node/Terminal_msg.h"
#include "ros_adapter_node/TerminalConfig.h"
#include "ros_adapter_node/AvoidOperation.h"
#include "ros_adapter_node/CarState.h"
#include "ros_adapter_node/QRInfo.h"
#include "ros_adapter_node/ErrorCode.h"
#include "ros_adapter_node/ErrorSet.h"
#include "ros_adapter_node/ActionList.h"
#include "ros_adapter_node/ActionFeedback.h"
#include "ros_adapter_node/AgvAlarmEntry.h"
#include "ros_adapter_node/AgvAlarmSet.h"
#include "ros_adapter_node/AgvStatus.h"
#include "ros_adapter_node/QRerrorArrived_msg.h"
#include "ros_adapter_node/CarButtonEvent.h"
#include "ros_adapter_node/gpio.h"
#include "ros_adapter_node/rfid.h"
#include "ros_adapter_node/PositionInfo.h"
#include "ros_adapter_node/RunningInfo.h"
#include "ros_adapter_node/SystemInfo.h"
#include "ros_adapter_node/TaskInfo.h"
#include "ros_adapter_node/AlarmInfo.h"
#include "ros_adapter_node/Collaborative2Camel.h"
#include "ros_adapter_node/Camel2Collaborative.h"
#include "ros_adapter_node/LoadMap.h"
#include "ros_adapter_node/Paths.h"
#include "ros_adapter_node/AddRoute.h"
#include "ros_adapter_node/checkPalletDis.h"
#include "ros_adapter_node/FetchCarHardwareDetails.h"
#include "ros_adapter_node/Cargo_msg.h"
#include "ros_adapter_node/Active_safety_enable_msg.h"
#include "std_srvs/SetBool.h"
#include "ros_adapter_node/position_check.h"
#include "ros_adapter_node/qr_station_info.h"
#include "RosAdapter.h"
#include "ControllerCommon.h"

namespace camelrosmsg
{
	class CRosAdapterImpl : public CRosPublish, public CRosConfigHandle 
	{
		enum PublishTopic
		{
			PT_Start,
			PT_AGVGoal,
			PT_ProhibitedArea,
			PT_AccessibleArea,
			PT_InitPose,
			PT_Camel2PlcNotify,
			PT_Camel2Plc,
			PT_Camel2PlcActionReq,
			PT_Odom,
			PT_Camel2Cbase,
			PT_Camel2Cpoints,
			PT_Camel2Cterminal,
			PT_Camel2CterminalConfig,
			PT_Camel2PlcStick,
			PT_Camel23DCargoAttribute,
			PT_Camel23DActiveSafeEnable,
			PT_ActionList,
			PT_AgvAlarm,
			PT_AgvStatus,
			PT_CarButtonEvent,
			PT_Camel2Collaborative,
			PT_AgvPath,
			PT_AgvStatusInfo,
			ST_End
		};

		enum ServiceType
		{
			Srv_Start,
			Srv_TerminalPosition,
			Srv_LoadMap,
			Srv_DrawFlag,
			Srv_CheckSpace,
			Srv_AddRoute,
			Srv_CheckPalletDis,
			Srv_FetchCarHardwareDetails,
			Srv_IDooCheckPalletDis,
			Srv_CheckPosision,
			Srv_QrStationInfo,
			Srv_End
		};

	public:
		CRosAdapterImpl() : spnr_(4), psubhandle(nullptr){};
		~CRosAdapterImpl(){};

		void Init(ros::NodeHandle n);
		static CRosAdapterImpl* GetInstanse();

		virtual void Publish(const std::string &topic, void *pmsg);

		virtual void BindSubscribe(CSubscribeHandle *subs);

		virtual void Release();

		virtual bool CallService(const std::string &service, void *prequest, void *presponse);

		virtual bool HasConfig(const std::string& key);
		virtual bool GetConfig(const std::string& key, std::string& value);
		virtual bool GetConfig(const std::string& key, int& value);
		virtual bool GetConfig(const std::string& key, bool& value);
		virtual bool GetConfig(const std::string& key, float& value);
		virtual bool SetConfig(const std::string& key, const std::string& value);
		virtual bool SetConfig(const std::string& key, const int& value);
		virtual bool SetConfig(const std::string& key, const bool& value);
		virtual bool SetConfig(const std::string& key, const float& value);
		virtual bool GetConfig(const std::string& key, std::map<std::string, bool>& maps);

	private:
		// 将camel要发布的消息封装成ros消息
		double QuaternionToRPY(const geometry_msgs::Quaternion &quaternion);
		void EncodeRosAgvGoalMsg(AGVGoal *&pmsg, ros_adapter_node::Navigation &rosmsg);
		void EncodeRosInitPoseMsg(CarPose *&pmsg, geometry_msgs::PoseWithCovarianceStamped &rosmsg);
		void EncodeRosTerminalMsg(TerminalRequest *&pmsg, ros_adapter_node::TerminalPosition &rosmsg);
		void EncodeRosOdomMsg(CamelOdom *&pmsg, nav_msgs::Odometry &rosmsg);
		void EncodeRosCamel2CBaseMsg(Camel::Controller::CINBASE *&pmsg, ros_adapter_node::State_msg &rosmsg);
		void EncodeRosCamel2CPointsMsg(std::vector<Camel::Controller::CPOINT> *&pmsg, ros_adapter_node::Point_vector_msg &rosmsg);
		void EncodeRosCamel2CTerminalMsg(Camel::Controller::RELATIVE_POS *&pmsg, ros_adapter_node::Terminal_msg &rosmsg);
		void EncodeRosCamel2CTerminalConfig(Camel::Controller::TERMINAL_CONFIG *&pmsg, ros_adapter_node::TerminalConfig &rosmsg);
		void EncodeAgvStatusInfo(AGVStatusInfo* pmsg, ros_adapter_node::AGVStatusInfo& rosmsg);
		// void EncodeRosCamelCarStateMsg(CamelCarState*& pmsg, ros_adapter_node::CarState& rosmsg);
		void EncodeActionList(ActionList *pmsg, ros_adapter_node::ActionList &rosmsg);
		void EncodeAgvAlarm(std::vector<AgvAlarm> *&pmsg, ros_adapter_node::AgvAlarmSet &rosmsg);
		void EncodeAgvStatus(CamelCarState *&pmsg, ros_adapter_node::AgvStatus &rosmsg);
		void EncodeCarButtonEvent(CarButtonEvent *pmsg, ros_adapter_node::CarButtonEvent &rosmsg);
		void EncodeCamel2Collaborative(Camel2Collaborative *pmsg, ros_adapter_node::Camel2Collaborative &rosmsg);
		// camel订阅消息的回调处理函数
		void LocalizationResultCallback(const ros_adapter_node::LocalizationResult::ConstPtr &pmsg);
		void PLCToCamelNotifyCallback(const ros_adapter_node::plc_to_camel::ConstPtr &pmsg);

		void PLCToCamelDirectCallback(const ros_adapter_node::plc_to_camel_direct::ConstPtr &pmsg);
		void PLCToCamelActionRspCallback(const ros_adapter_node::plc_to_camel::ConstPtr &pmsg);
		void CToCamelMsgCallback(const ros_adapter_node::Controller2Camel_msg::ConstPtr &pmsg);
		void QRCameraCheckMsgCallback(const ros_adapter_node::QRInfo::ConstPtr &pmsg);
		void QRCameraMsgCallback(const ros_adapter_node::QRInfo::ConstPtr &pmsg);
		void ROSCamelAlarmMsgCallback(const ros_adapter_node::ErrorSet::ConstPtr &pmsg);
		bool AvoidOperationRequestCallback(ros_adapter_node::AvoidOperation::Request &req, ros_adapter_node::AvoidOperation::Response &rsp);
		bool GetCarStateCallback(ros_adapter_node::CarState::Request &req, ros_adapter_node::CarState::Response &rsp);
		void ActionFeedbackCallback(const ros_adapter_node::ActionFeedback::ConstPtr &pmsg);
		void MarkerLRCallback(const ros_adapter_node::LocalizationResult::ConstPtr &pmsg);
		void QrErrorArrivedCallback(const ros_adapter_node::QRerrorArrived_msg::ConstPtr &pmsg);
		void Collaborative2CamelCallback(const ros_adapter_node::Collaborative2Camel::ConstPtr &pmsg);
		void RfidInfoCallback(const ros_adapter_node::rfid::ConstPtr &pmsg);
		void GpioInfoCallback(const ros_adapter_node::gpio::ConstPtr &pmsg);
		bool AlarmInfoServiceCallback(ros_adapter_node::AlarmInfo::Request &req, ros_adapter_node::AlarmInfo::Response &rsp);
		bool PositionInfoServiceCallback(ros_adapter_node::PositionInfo::Request &req, ros_adapter_node::PositionInfo::Response &rsp);
		bool RunningInfoServiceCallback(ros_adapter_node::RunningInfo::Request &req, ros_adapter_node::RunningInfo::Response &rsp);
		bool SystemInfoServiceCallback(ros_adapter_node::SystemInfo::Request &req, ros_adapter_node::SystemInfo::Response &rsp);
		bool TaskInfoServiceCallback(ros_adapter_node::TaskInfo::Request &req, ros_adapter_node::TaskInfo::Response &rsp);
		void AgvPathCallback(const ros_adapter_node::Paths::ConstPtr &pmsg);

		ros::NodeHandle nh_;

		ros::AsyncSpinner spnr_;
		CSubscribeHandle *psubhandle;
		static std::atomic<unsigned int> ros_req_id;
		std::map<std::string, ros::Publisher> map_pub_handle_;		   //<发布的话题,发布句柄>
		std::map<std::string, PublishTopic> map_pub_type_;			   // <发布的话题,话题对应的枚举> 用于switch case
		std::map<std::string, ros::Subscriber> map_sub_handle_;		   // <订阅的话题,订阅句柄>
		std::map<std::string, ros::ServiceClient> map_service_client_; // <作为客户端的服务,客户端句柄>
		std::map<std::string, ServiceType> map_service_name_;		   // <作为客户端的服务,服务对应的枚举> 用于switch case
		std::map<std::string, ros::ServiceServer> map_service_server_; //<作为服务端的服务,服务端句柄>
	};

}
