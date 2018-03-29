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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_ECU_Data"); //node name
  ros::NodeHandle nh;
  ros::Publisher pubECUdata;
  pubECUdata = nh.advertise<std_msgs::Float32>("huachen_ecu_data",1);
  ros::Rate rate(10); //发布频率

  //get Nport data and analysis them
  CAnalysisECU m_AnalysisECU;
  int listen_port = 9002;
  if(!m_AnalysisECU.Init(listen_port))
  {
    printf("[ERROR] Cannot build connection to ECU!\n");
    return -1;
  }

  while(ros::ok())
  {

    m_AnalysisECU.Update();
    std_msgs::Float32 ecu_msg;
    ecu_msg.data = m_AnalysisECU.ECUData_struct.fForwardVel;
    pubECUdata.publish(ecu_msg);
    rate.sleep();
  }
}

