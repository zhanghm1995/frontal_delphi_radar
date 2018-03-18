#pragma once

//每一个毫米波雷达目标的数据结构体
typedef struct _moving_object_millimeter{
	unsigned short target_ID; //目标ID号，从1-64
	double range;			//目标距离
	double v;				//目标速度
	double angle;			//目标方位角
	double x;
	double y;
	bool valid;
	//zhanghm:20170911 new add
	unsigned short status;//目标状态,CAN_TX_TRACK_STATUS
	unsigned short moving; //运动状态
	bool moving_fast;
	bool moving_slow;
}moving_object_millimeter;

typedef struct _delphi_radar_target{
	moving_object_millimeter delphi_detection_array[64]; //64个目标数据
	unsigned short ACC_Target_ID;//ACC目标的ID号,0-64,0意味着没有ACC目标
}delphi_radar_target;


//Vehicle information used to send to MMW Radar
typedef struct Vehicle_Info_
{
	float yaw_rate; //degree/s
	float vehicle_speed;//Unit: m/s
	float steering_angle;// Unit: degree
}Vehicle_Info;
