#ifndef CAMEL_CONTROLLER_COMMON_H
#define CAMEL_CONTROLLER_COMMON_H
/*
 * windows版本运动控制作为camel的一个库，MCU版本上运动控制独立为一个ROS节点
 * 两个版本camel和运动控制通信的消息结构不完全一致，为了兼容，
 * camel内部使用该文件定义的结构（兼容两个版本的所有字段），
 * 仅在ControllerService中使用运动控制的头文件（windows版本）
 * 在与运动控制通信时，再根据操作系统对消息进行转换
 */
#include <memory>
#include <vector>
#ifdef _LINUX64
#include <string.h>
#endif
namespace Camel{
namespace Controller {

	enum RunMode
	{
		kManual,
		kDebug,
		kAuto
	};

	//车型定义
	enum EVehicleModel
	{
		VM_Stacker = 0,       //堆垛车   --- 叉车(L16, MR15)
		VM_PalletTruck,       //托盘车   --- kiva
		VM_Omnidirectional,   //全向车   --- 双舵轮		
	};

	//发给C的定点旋转命令
	typedef struct tag_c_rotate
	{
		uint8_t		rotate_type;//原地旋转动作类型，0关闭，1带目标角度的原地旋转，2不带目标角度的原地盲转
		float		rotate_goal;//原地旋转目标角度
		tag_c_rotate() { memset(this, 0, sizeof * this); }
		tag_c_rotate(uint8_t type, float goal)
			: rotate_type(type), rotate_goal(goal)
		{ }
	}CROTATE, *PCROTATE;

	// C 输入基础
	typedef struct tag_c_in_base
	{
		tag_c_in_base() { memset(this, 0, sizeof * this); }
		tag_c_in_base(bool _enable, uint16_t _status, uint8_t _mode, 
			uint8_t _vehicleModel, uint8_t _dataClear, uint32_t _id,
			uint8_t _cargo, uint8_t _rotate_type, float _rotate_goal)
			: enable(_enable)
			, status(_status) 
			, mode(_mode)
			, vehicleModel(_vehicleModel)
			, dataClear(_dataClear)
			, id(_id)
			, cargo(_cargo)
		{
		}

		bool		enable;		// C程序是否运行的使能信号 IO层上报过来的使能信号
		uint16_t	status;		// 状态字
		uint8_t		mode;		// 运行模式 0:手动 1:调试  2:自动
		uint8_t		vehicleModel;	//EVehicleModel
		uint8_t     dataClear;  //路线数据清空，1使能，0关闭
		uint32_t    id;
		uint8_t 	cargo;  //有货/无货标记
		CROTATE		rotate;
		uint8_t		controlMode;// 控制模式(0 B样条, 1 磁导航)

	} CINBASE, *PCINBASE;

	typedef struct tag_coord
	{
		tag_coord() { memset(this, 0, sizeof * this); }
		tag_coord(float _x, float _y, float _theta)
			: x(_x)
			, y(_y)
			, theta(_theta)
		{
		}

		float x;
		float y;
		float theta;
	} COORD, *PCOORD;

	//终点类型
	enum EDestType
	{
		DT_Normal = 0,     //普通站点
		DT_Traffic,        //普通管制点
		DT_Navichange,     //导航方式切换点
		DT_Tail,           //路线终点
		DT_Get,            //路线终点且是取货点
		DT_Put,            //路线终点且是放货点
	};

	//Camel->C
	typedef struct tag_c_point
	{
		tag_c_point() { memset(this, 0, sizeof * this); }

		tag_c_point(float _x, float _y, float _theta, float _velocity,
			float _radius, uint16_t _method, uint32_t _QR_id, uint8_t _destype = 0)
			: x(_x),
			y(_y),
			theta(_theta),
			velocity(_velocity),
			radius(_radius),
			destype(_destype),
			controlMethod(_method),
			QR_id(_QR_id)
		{}

		float x;	    // x坐标
		float y;	    // y坐标
		float theta;    // 角度   路线上 goals_angle
		float velocity; // 速度
		float radius;   // 圆弧半径 路线上的
		uint8_t destype;// 终点类型EDesType
		uint16_t controlMethod; // 0 : B样条，1 : 优先跟踪其次自主规划
		uint32_t QR_id;    // 二维码id
	} CPOINT, *PCPOINT;

	// mcu speed
	typedef struct tag_ttlv_mcu_v_speed
	{
		tag_ttlv_mcu_v_speed() { memset(this, 0, sizeof * this); }
		tag_ttlv_mcu_v_speed(float x, float y, float w, float _rad) : Vx(x), Vy(y), W(w), rad_steering(_rad) {}
		tag_ttlv_mcu_v_speed(const tag_ttlv_mcu_v_speed& rhs)
			:Vx(rhs.Vx), Vy(rhs.Vy), W(rhs.W), rad_steering(rhs.rad_steering)
		{}

		tag_ttlv_mcu_v_speed& operator = (const tag_ttlv_mcu_v_speed& rhs)
		{
			if (this != &rhs)
			{
				this->Vx = rhs.Vx;
				this->Vy = rhs.Vy;
				this->W = rhs.W;
				this->rad_steering = rhs.rad_steering;
			}
			return *this;
		}

		float Vx;			// 实时速度 x轴 单位 mm/s
		float Vy;
		float W;
		float rad_steering; // 舵轮角度 added by jl.xie @ 2019.12.2
	}MCU_V_SPEED, *PMCU_V_SPEED;

	//相对位置，用于二维码、末端定位
	typedef struct tag_relative_pos
	{
		tag_relative_pos() { memset(this, 0, sizeof * this); }
		tag_relative_pos(bool enable, bool isvalid, float x, float y, float theta, float derror, float aerror, int type, int check,int expect,int real) : terminal_enable(enable), isdatavalid(isvalid),deltax(x), deltay(y), deltatheta(theta), distance_maxerror(derror), angle_maxerror(aerror), terminal_type(type), check_point(check),expect_ID(expect),real_ID(real) {}
		bool  terminal_enable; //末端定位使能
		bool  isdatavalid;     //数据是否有效
		float deltax;
		float deltay;
		float deltatheta;
		float distance_maxerror;//最大距离误差
		float angle_maxerror;  //最大角度误差
		int   terminal_type;   //末端定位类型
		int   check_point;     //确认点，1有数据确认，2无数据确认
		int   expect_ID;
		int   real_ID;
	}RELATIVE_POS;
	
	// 给controller发送末端定位 的配置信息
	typedef struct tag_terminal_config
	{
		tag_terminal_config() { memset(this, 0, sizeof * this); }
		tag_terminal_config(int type, float x, float y, float theta) : target_type(type), deltax(x), deltay(y), deltatheta(theta) {}
		int  target_type; // 类型
		float deltax;
		float deltay;
		float deltatheta;
	}TERMINAL_CONFIG;

	typedef struct Camel2C
	{
		CINBASE						base;		// 基础信息
		COORD						coord;		// 实时坐标
		std::vector<tag_c_point>	VPOINT;     // 站点
		MCU_V_SPEED                 mcuSpeed;   // mcu 实时速度和舵轮速度
		RELATIVE_POS                rpos;       //相对位置

		char* ToString()
		{
			static char szLog[2048] = { 0 };
			memset(szLog, 0, 2048);
			int logLen = sprintf(szLog,
				"base:enable.status[%d, %d], "
				"coord[%.2f, %.2f, %.2f], ", base.enable, base.status, coord.x, coord.y, coord.theta);

			char szPoints[512] = { 0 };
			int pointsLen = 0;
			std::vector<tag_c_point>::iterator iter;

			for (iter = VPOINT.begin(); iter != VPOINT.end(); iter++)
			{
				char szTmp[512] = { 0 };
				int len = 0;

				len = sprintf(szTmp, "\tVPOINT:[%.2f, %.2f, %.2f]\t", (*iter).x, (*iter).y, (*iter).theta);
				memcpy(szPoints + pointsLen, szTmp, len);
				pointsLen += len;
			}

			memcpy(szLog + logLen, szPoints, pointsLen);

			return szLog;
		}
	} CAMEL2C, *PCAMEL2C;

	//C->Camel
	typedef struct C2Camel
	{
		C2Camel() { memset(this, 0, sizeof * this); }
		C2Camel(float _Vx, float _Vy, float _w, uint16_t _status, float _od1, 
			float _od2, uint16_t _indexFromEnd, uint8_t _dataClearDone, 
			uint8_t _autoObsAvoid, uint8_t _terminal_status, uint8_t _cir_Flag, 
			uint8_t _rotate_finished, uint8_t _typevw, uint32_t _id)
			: Vx(_Vx)
			, Vy(_Vy)
			, w(_w)
			, status(_status)
			, od1(_od1)
			, od2(_od2)
			, indexFromEnd(_indexFromEnd)
			, dataClearDone(_dataClearDone)	
			, autoObsAvoid(_autoObsAvoid)
			, cir_Flag(_cir_Flag)
			, terminal_status(_terminal_status)
			, rotate_finished(_rotate_finished)
			, typevw(_typevw)
			, id(_id)
		{
			memset(this->alarmCode, 0, sizeof alarmCode);
			memset(&(this->coord), 0, sizeof(COORD));
		}

		float		Vx;
		float		Vy;
		float		w;
		uint16_t	status;
		uint16_t	alarmCode[5];
		float		od1;
		float		od2;
		int16_t		indexFromEnd;	//从通行站点往起始站点的索引，从0开始，0表示站点是通行站点，默认值为-1
		uint8_t     dataClearDone;  //数据清空完成，1完成，0未完成
		uint8_t     autoObsAvoid;   //自动避障状态，0默认值，1自动避障完成，2自动避障失败
		uint8_t     cir_Flag;   // 圆弧标记，用于原地转弯
		uint8_t     terminal_status;//末端定位状态，默认为0，1表示终点校验通过结束末端定位，2表示异常结束末端定位
		uint8_t     rotate_finished;//原地旋转动作完成状态，0未完成，1完成
		uint8_t     typevw;         //转弯类型，0直线，1车头朝前左转，2车头朝前右转，3叉子朝前左转，4叉子朝前右转
		uint32_t    id;
		COORD		coord;		// index=0站点坐标


		char* ToString()
		{
			static char szLog[2048] = { 0 };
			memset(szLog, 0, 2048);
			sprintf(szLog,
				"motion[%.2f,%.2f,%.2f], "
				"status[%d], "
				"alarm[%d,%d,%d,%d,%d],"
				"od[%.2f, %.2f],"
				"_indexFromEnd[%d]"
				, Vx, Vy, w, status, alarmCode[0], alarmCode[1], alarmCode[2], alarmCode[3], alarmCode[4], od1, od2, indexFromEnd);

			return szLog;
		}
	} C2CAMEL, *PC2CAMEL;
}

}
#endif // !CAMEL_CONTROLLER_COMMON_H
