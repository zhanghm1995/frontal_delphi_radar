
#include <dlfcn.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <string.h>
#include <netinet/in.h>
#include <math.h>
#include <fstream>
#include <unistd.h>
#include  <iostream>
#define HUACHEN
#define BUFSIZE 4096

struct struct_ECU			// 车辆状态
{
	/**********状态变量*********/
	double fForwardVel;					// 车辆纵向速度(Unit:m/s)
	double fDeForwardVel;
	double fFLRWheelAverAngle;			// 名义前轮偏角，对应电机或方向盘的角度(Unit:°)
	double fOdometer;
	double fRadius;

	unsigned char f_shift;				//档位 0无效1P2R3N4N5D6M7S8+9-
	unsigned char f_shift1;				//具体档位
	unsigned char f_estop;				//紧急制动
	unsigned char f_leftlamp;			//左转向灯
	unsigned char f_rightlamp;			//右转向灯
	bool lateralctrl_enabled;
	bool longitutdectrl_enabled;
	bool brake_enabled;//人工制动踏板是否被踩下 qjy 20180125
	double	lTimeStamp;			// 时间戳(Unit:ms)

	char autodrive_status;   //add
	char brake_pedal_signal;  //add
	char switch_signal;  //add


	double pressure_back;         //后油路压力
	double petral_pressure;       //制动踏板压力

	int throtle_feedback;
	char steerRx_err;
	char steerTx_err;
	char brakeRx_err;
	char brakeTx_err;
	char PC_Tx_err;

	char poweron_status;
	char start_status;
	char warning_status;
	char bugle_status;

	char light_far;
	char light_near;

	char Estop_enabled;

	int EnginRate;
	//qjy add 0918
	double FrontLeftWheelSpeed;
	double FrontRightWheelSpeed;
	double BackLeftWheelSpeed;
	double BackRightWheelSpeed;
	//*************************GHJ20171012增加车辆底层反馈信息*************************//
	unsigned char FrontLeftWheelSpeedStatus;
	unsigned char FrontRightWheelSpeedStatus;
	unsigned char BackLeftWheelSpeedStatus;
	unsigned char BackRightWheelSpeedStatus;

	double Yaw_Rate;
	double Yaw_Rate_Offset;
	unsigned char Yaw_Rate_Status;

	double AX;
	double AX_Offset;
	unsigned char AX_Status;

	double AY;
	double AY_Offset;
	unsigned char AY_Status;
	//*************************GHJ20171012增加车辆底层反馈信息*************************//
    //*************************霍钊20180103增加丰田底层反馈信息*************************//
//    double T_lBrakePressure;
//    double T_rBrakePressure;
    double T_throttlePedalPosition1;
    double T_throttlePedalPosition2;
    double T_lFBrakePressure;
    double T_rFBrakePressure;
    double T_lRBrakePressure;
    double T_rRBrakePressure;
    unsigned char T_escerrorCode;
    unsigned char T_bottomerrorCode;
    double Engine_load;
//    unsigned char f_shift_num_OBD;      //具体档位
};




class CAnalysisECU
{
   public:
   struct_ECU   ECUData_struct;
    CAnalysisECU(); 
    ~CAnalysisECU(); 
    bool Init(int port);
    void Update();
    double steeringratio_l;//qjy,0829
    double steeringratio_r;
  private:
    char m_ECUDataFromReceiver[60];
    void ECU_DataProcFromVehicle(unsigned char* data);
    double convert_ctrlvalue2steeringangle(int ctrlvalue , bool direction, double steeringratio_l, double steeringratio_r);

    sockaddr_in myaddr; /* our address */
    sockaddr_in remaddr; /* remote address */
    socklen_t addrlen; /* length of addresses */

    int recvlen; /* # bytes received */
    int fd; /* our socket */
    unsigned char buf[BUFSIZE]; /* receive buffer */
        double petral_pressure;

    //霍钊添加
      int CANFound;


//    std::fstream data_backup;
};
