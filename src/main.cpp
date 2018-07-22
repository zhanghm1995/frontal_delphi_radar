#include <iostream>
//ROS
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "sensor_driver_msgs/ECUData.h"//ECUData
//Boost
#include <boost/algorithm/string.hpp>
#include <boost/timer.hpp>
#include <boost/math/special_functions/round.hpp>
#include "boost/asio.hpp"
#include <boost/thread/thread.hpp>
//Project headers
#include "frontal_delphi_radar.h"
#include "object_detection_radar.h"
#include "frontal_delphi_radar/RadarPoint.h"
#include "frontal_delphi_radar/RadarData.h"
using namespace std;

//subscribe ROS messages and implement the detph image conversion
class PostProcess
{
public:
  PostProcess(ros::NodeHandle& nodehandle):nodehandle_(nodehandle),
      //      image_transport_nh_(nodehandle), //for transport image
      processthread_(NULL),
      processthreadfinished_ (false)
  {
    memset(&radar_data_,0,sizeof(radar_data_));
    init();
  }
  ~PostProcess()
  {
    processthreadfinished_ = true;
    processthread_->join();
  }

  void init()
  {
    subRadarData_ = nodehandle_.subscribe<frontal_delphi_radar::RadarData>("radardata", 1, boost::bind(&PostProcess::RadarDataHandler,this,_1));//
    processthread_ = new boost::thread(boost::bind(&PostProcess::process,this));
  }

  void RadarDataHandler(const frontal_delphi_radar::RadarDataConstPtr& radar_msg)
  {

//    ROS_INFO("<main> radar data callback...");
    for(int i = 0;i<64;++i)
    {
      radar_data_.delphi_detection_array[i].target_ID = radar_msg->delphi_detection_array[i].target_ID;
      radar_data_.delphi_detection_array[i].range = radar_msg->delphi_detection_array[i].range;
      radar_data_.delphi_detection_array[i].v = radar_msg->delphi_detection_array[i].v;
      radar_data_.delphi_detection_array[i].x = radar_msg->delphi_detection_array[i].x;
      radar_data_.delphi_detection_array[i].y = radar_msg->delphi_detection_array[i].y;
      radar_data_.delphi_detection_array[i].angle = radar_msg->delphi_detection_array[i].angle;
      radar_data_.delphi_detection_array[i].valid = radar_msg->delphi_detection_array[i].valid;
      radar_data_.delphi_detection_array[i].status = radar_msg->delphi_detection_array[i].status;
      radar_data_.delphi_detection_array[i].moving = radar_msg->delphi_detection_array[i].moving;
      radar_data_.delphi_detection_array[i].moving_fast = radar_msg->delphi_detection_array[i].moving_fast;
      radar_data_.delphi_detection_array[i].moving_slow = radar_msg->delphi_detection_array[i].moving_slow;
    }
    radar_data_.ACC_Target_ID = radar_msg->ACC_Target_ID;
    radar_data_.ESR_vehicle_speed = radar_msg->ESR_vehicle_speed;//ego vehicle speed
    radar_data_.ESR_yaw_rate = radar_msg->ESR_yaw_rate; //ego vehicle yaw angle rate
    radar_data_.vehicle_speed_origin = radar_msg->vehicle_speed_origin;
  }
  void process()
  {

    object_detection_.draw_basic_info();//绘制网格线
    while(!processthreadfinished_)
    {
      //data visualizition
      object_detection_.set_radar_data(radar_data_);//传递获取的雷达数据
      object_detection_.main_function2();
      object_detection_.DisplayAll();//显示图像
    }

  }



private:
  ros::NodeHandle& nodehandle_;
  ros::Subscriber subRadarData_ ;//sub vehicle speed
  //multi thread
  boost::thread* processthread_;
  bool processthreadfinished_;
  //zhanghm add: 20180129
  delphi_radar_target radar_data_;
  ObjectDetection object_detection_;
  Vehicle_Info vehicle_info_received_; //收到的车辆状态信息
};

int main(int argc, char** argv){
  ros::init(argc, argv, "frontal_delphi_radar");
  ros::NodeHandle nh;

  PostProcess postprocess(nh);

  ros::spin();
}
