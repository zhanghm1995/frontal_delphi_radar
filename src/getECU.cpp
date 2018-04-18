/******************************************************************************
 * Author: Haiming Zhang
 * Email : zhanghm_1995@qq.com
 * Description: get huachen ECU data
 *
 *****************************************************************************/

#include <time.h>
//ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>

//project headers
#include "AnalysisECU.h"

std::string vehicle_name;//车辆类型
int listen_port; //ECU监听端口
int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_ECU_Data"); //node name
  ros::NodeHandle nh;
  ros::Publisher pubECUdata;
  pubECUdata = nh.advertise<std_msgs::Float32>("ecu_data",1);
  ros::Rate rate(10); //发布频率

  //get Nport data and analysis them
  CAnalysisECU m_AnalysisECU;

  //get parameters
  nh.param<std::string>("vehicle_name",vehicle_name,"BYD_TANG");
  nh.param<int>("listen_port",listen_port,9001);

  bool conect_flag = false;
  if(vehicle_name == "BYD_TANG")
  {
	  conect_flag=m_AnalysisECU.Init(CAnalysisECU::BYD_TANG,listen_port);
  }
  else if(vehicle_name == "HUACHEN")
  {
	  conect_flag=m_AnalysisECU.Init(CAnalysisECU::HUACHEN,listen_port);
  }
  if(!conect_flag)
  {
    printf("[ERROR] Cannot build connection to ECU!\n");
    return -1;
  }
  ROS_INFO("Connection to ECU");

  while(ros::ok())
  {
    m_AnalysisECU.Update();
    std_msgs::Float32 ecu_msg;
    ecu_msg.data = m_AnalysisECU.ECUData_struct.fForwardVel;
    pubECUdata.publish(ecu_msg);
    printf("vehicle speed is %.3f\n",m_AnalysisECU.ECUData_struct.fForwardVel);
//    rate.sleep();
  }
}

