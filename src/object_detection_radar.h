/*==================================================================
 *    功能    ：  毫米波雷达数据可视化
 *                  1、动态目标检测
                   2、静态目标检测
 *    相关说明：
 *    作者    ：  zhanghm
 *    创建日期    ：20180307
 *    修改记录：
/*==================================================================*/
#ifndef OBJECT_DETECTION_RADAR_H_
#define OBJECT_DETECTION_RADAR_H_
//C&C++ headers
#include <cstdio>
#include <vector>
#include <algorithm>
//OpenCV
#include <opencv2/opencv.hpp>
//project headers
#include "TypeDef.h"

using std::vector;

#define NUM 64  //毫米波雷达总的检测目标数
#define METER2PIXEL 5 //1米为5个像素点，根据相机标定参数来定,每个像素间隔0.2m，即像素分辨率为20cm。
#define OBJECT_WIDTH 2 //默认实际车宽为2米
#define PLANE_WIDTH 401  //鸟瞰图
#define PLANE_HEIGHT 601

#define SHOW_TARGET_INFO //显示目标属性信息
#define DISTANCE_TO_RADAR //鸟瞰图中距离标尺为到雷达的距离

//可变参数
#define RADAR2CAR 1.5 //毫米波到车后轴距离，米制单位
#define XLim 2.2//感兴趣目标点在毫米波雷达x正方向最大距离限制

#define SAVE_GET_DATA //是否保存获得的待处理的毫米波数据和自身车辆信息


const int persMap_Middle = (PLANE_WIDTH - 1) / 2; //鸟瞰图宽度中点坐标
const int T1 = 15; //历史设的T1，T2都是20
const int T2 = 20;


class ObjectDetection{
public:
  ObjectDetection(void);
  ~ObjectDetection(void);

public:
  //类数据获取
  void set_radar_Data(delphi_radar_target& radar,Vehicle_Info& _vehicle_info);//在线获取毫米波数据和车辆信息
  void set_radar_data(const delphi_radar_target& radar);//在线获取毫米波数据

  void show_result(moving_object_millimeter& obj_interest); //得到最终显示画面
  void show_result(vector<moving_object_millimeter>& valid_obj);//显示全部有效目标
  void ShowACCTarget(const int& ACC_ID); //单独显示ACC目标

  void compare(); //生成单独不用算法的ACC目标，对比效果
  void draw_basic_info(); //绘制基本信息
  void main_function(); //主处理函数

  //zhanghm-20180304-毫米波雷达数据可视化
  void main_function2(); //

  moving_object_millimeter getSendData(); //获得最终需要发送的目标

private:
  //功能函数


  //函数对象，供vector查找用
  bool operator()(const moving_object_millimeter& obj) const
  {
      if(obj.target_ID == m_ACC_ID)
          return true;
      else
          return false;
  }


  //成员变量
public:
  bool save_flag;
  IplImage* m_Delphi_img_bak; //用于存储绘制好基本要素的雷达图
  IplImage* m_Delphi_img; //雷达点鸟瞰图
  IplImage* m_Obj_Interest_img; //感兴趣目标雷达鸟瞰图，此处指正前方目标

  //从雷达获取的信息
  unsigned short m_ACC_ID;
  moving_object_millimeter delphi_detection_array[NUM];//获取的毫米波雷达目标数据
  Vehicle_Info vehicle_info_; //获得的自身车辆信息
  moving_object_millimeter final_obj;
  FILE* fpp;
  //实际处理的雷达和车辆信息数据
  FILE* fp_radar_file;
  FILE* fp_vehicle_file;

  char* text; //用来显示各种文本
  CvFont cf;
  CvFont cf2; //计时用字体

  vector<moving_object_millimeter> vecObj_Moving;
  vector<moving_object_millimeter> vecObj_Temp;
  vector<moving_object_millimeter> vecObj_Interest;
  vector<moving_object_millimeter> vecObj_ACC;
  vector<double> vecObj_Distance;




  int m_frameCount;
  int m_ACC_Count; //判断连续获得ACC目标的帧数
  unsigned short m_ACC_Record[2]; //记录上一帧和当前帧ACC的ID号
  int _count_num;
  int _recvNum; //接收数据次数
  vector<unsigned short> vecACC_ID;
  bool m_first_ACC_flag; //用来标志是否第一次获得ACC目标
  bool m_full_ACC_flag; //已有连续的满足指定帧数的ACC目标

private:
  moving_object_millimeter _empty_obj;



};

#endif /*OBJECT_DETECTION_RADAR_H_*/
