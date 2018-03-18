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
    memset(&vehicle_info_received_,0,sizeof(vehicle_info_received_));
    init();
  }
  ~PostProcess()
  {
    processthreadfinished_ = true;
    processthread_->join();
  }

  void init()
  {
    subECUData_ = nodehandle_.subscribe<sensor_driver_msgs::ECUData>("ecudata", 1, boost::bind(&PostProcess::ECUDataHandler,this,_1));//
    subImuData_ = nodehandle_.subscribe<sensor_msgs::Imu>("imudata", 1, boost::bind(&PostProcess::ImuDataHandler,this,_1));//
    processthread_ = new boost::thread(boost::bind(&PostProcess::process,this));
  }




  void ECUDataHandler(const sensor_driver_msgs::ECUDataConstPtr& ecu_data_msg) //EUC数据
  {
    vehicle_info_received_.vehicle_speed = ecu_data_msg->fForwardVel;//车速, m/s

  }
  void ImuDataHandler(const sensor_msgs::ImuConstPtr& imu_data_msg)
  {
    vehicle_info_received_.yaw_rate = (imu_data_msg->angular_velocity.z)*180.0/3.1415926; //横摆角速度, degree/s
  }

  void process()
  {

    object_detection_.draw_basic_info();
    bool flag = frontal_delphi_receiver_.Init();//build connection to MMW radar
    if(flag == true){
      cout<<"[INFO] MMW Radar UDP socket has been built!"<<endl;
    }
    else{
      cout<<"error"<<endl;
      return;
    }
    while(!processthreadfinished_)
    {
      frontal_delphi_receiver_.set_self_vehicle_info(vehicle_info_received_);
      frontal_delphi_receiver_.Update();
      //data visualizition
      delphi_radar_target radar_data=frontal_delphi_receiver_.radar_target_data();
      object_detection_.get_radar_Data(radar_data);
      object_detection_.main_function2();
      IplImage* delphi_image = object_detection_.m_Delphi_img;
      cvNamedWindow("delphi_image",CV_WINDOW_NORMAL);
      cvShowImage("delphi_image", delphi_image);

      int key = cvWaitKey(10);
      if(key == 32)
        cvWaitKey(0);
    }

  }



private:
  ros::NodeHandle& nodehandle_;
  ros::Subscriber subImuData_ ;//sub yaw rate
  ros::Subscriber subECUData_ ;//sub vehicle speed
  //multi thread
  boost::thread* processthread_;
  bool processthreadfinished_;
  //zhanghm add: 20180129
  FrontalDelphiRadar frontal_delphi_receiver_;
  ObjectDetection object_detection_;
  Vehicle_Info vehicle_info_received_; //收到的车辆状态信息
};

int main(int argc, char** argv){
  ros::init(argc, argv, "depth_image_utils");
  ros::NodeHandle nh;

  PostProcess postprocess(nh);

  ros::spin();
}
