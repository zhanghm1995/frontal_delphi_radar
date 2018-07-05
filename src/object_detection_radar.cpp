#include "object_detection_radar.h"
#define PI 3.1415926535898
const double toRAD = PI/180.0;

bool comp_by_y(const moving_object_millimeter &obj1,const moving_object_millimeter &obj2)
{
    return obj1.y<obj2.y; //y值小的排前面

}


class Find_ACC_ID
{
public:

    Find_ACC_ID(const unsigned short& id):ACC_ID(id){}
    bool operator()(const moving_object_millimeter& obj) const
    {
        if(obj.target_ID==ACC_ID)
            return true;
        else
            return false;
    }

private:
    unsigned short ACC_ID;


};


ObjectDetection::ObjectDetection(void)
{
    //图像成员变量指针内存申请
    m_Delphi_img_bak = cvCreateImage(cvSize(PLANE_WIDTH, PLANE_HEIGHT), IPL_DEPTH_8U, 3);
    m_Delphi_img = cvCreateImage(cvSize(PLANE_WIDTH, PLANE_HEIGHT), IPL_DEPTH_8U, 3);//雷达点鸟瞰图
    m_Delphi_img_compare = cvCreateImage(cvSize(PLANE_WIDTH, PLANE_HEIGHT), IPL_DEPTH_8U, 3);
    cvZero(m_Delphi_img_bak);
    cvZero(m_Delphi_img);
    cvZero(m_Delphi_img_compare);

    text = new char[260];
    //字体初始化
    cvInitFont(&cf, CV_FONT_HERSHEY_PLAIN, 0.8, 0.8, 0.0, 1);
    cvInitFont(&cf2, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0.0, 1);
    for(int i = 0; i<NUM; i++)
    {
        delphi_detection_array[i].target_ID = 0;
        delphi_detection_array[i].x = 0.0;
        delphi_detection_array[i].y = 0.0;
        delphi_detection_array[i].v = 0.0;
        delphi_detection_array[i].angle = 0.0;
        delphi_detection_array[i].range = 0.0;
        delphi_detection_array[i].valid = 0;
        delphi_detection_array[i].moving = 0;
        delphi_detection_array[i].moving_fast =0;
        delphi_detection_array[i].moving_slow = 0;
        delphi_detection_array[i].status =0;

    }

    _empty_obj.target_ID = 0;
    _empty_obj.x = 0;
    _empty_obj.y = 0;
    _empty_obj.v = 35.2778; //发送默认速度,35.2778m/s = 127km/h
    _empty_obj.angle = 0.0;
    _empty_obj.range = 200;
    _empty_obj.valid = 0;
    _empty_obj.moving = 0;
    _empty_obj.moving_fast = 0;
    _empty_obj.moving_slow = 0;
    _empty_obj.status = 0;

    final_obj = _empty_obj;



    m_frameCount = 0;
    m_ACC_Count = 0;
    m_first_ACC_flag = false;
    m_full_ACC_flag = false;
    _count_num = 0;
    _recvNum = 0;

    fp_radar_file = fopen("1_radar_data_handle.txt","w");
    //fprintf(fp_SaveData,"target_ID,status,angle,range,x,y");
    fp_vehicle_file = fopen("2_vehicle_data_handle.txt","w");
    fpp = fopen("3_ACC_Target.txt","w");
}

ObjectDetection::~ObjectDetection(void)
{


}

void ObjectDetection::draw_basic_info()
{
    //画车,用点代表本车
    cvCircle(m_Delphi_img_bak, cvPoint(200, 600), 5, cvScalar(0, 0, 255), 2);

    //画距离图，五个像素=实际距离1米
#ifdef DISTANCE_TO_RADAR //默认显示距离为到毫米波雷达的距离
    for (int h = 600-RADAR2CAR*METER2PIXEL, t = 0; (h-25) > 0; h -= 25, t += 5) //画横线
    {
        cvLine(m_Delphi_img_bak, cvPoint(0, h), cvPoint(400, h), cvScalar(122, 122, 122));
        sprintf(text, "%d m", t);
        cvPutText(m_Delphi_img_bak, text, cvPoint(5, h - 2), &cf, cvScalar(0, 255, 255));
    }
#else
    for (int h = 500, t = 0; h > 0; h -= 25, t += 5) //画横线
    {
        cvLine(m_Delphi_img_bak, cvPoint(0, h), cvPoint(400, h), cvScalar(122, 122, 122));
        sprintf(text, "%d m", t);
        cvPutText(m_Delphi_img_bak, text, cvPoint(5, h - 2), &cf, cvScalar(0, 255, 255));
    }
#endif
    for (int w = 50; w < 400; w += 50)  //画竖线
    {
        cvLine(m_Delphi_img_bak, cvPoint(w, 0), cvPoint(w, 600), cvScalar(122, 122, 122));
    }

    int vertical_distance_line = 1.5;//绘制左右竖线距离线
    int pixel_dis = cvRound(vertical_distance_line*METER2PIXEL);
    cvLine(m_Delphi_img_bak,cvPoint(200-pixel_dis,0),cvPoint(200-pixel_dis,600),CV_RGB(0,0,255));
    cvLine(m_Delphi_img_bak,cvPoint(200+pixel_dis,0),cvPoint(200+pixel_dis,600),CV_RGB(0,0,255));
//    cvShowImage("delphi_image", m_Delphi_img_bak);
//    cvWaitKey(50);


}

void ObjectDetection::set_radar_data(const delphi_radar_target& radar)
{
  m_ACC_ID=radar.ACC_Target_ID;
  vehicle_info_.vehicle_speed = radar.ESR_vehicle_speed; //从ESR读到的车速
  vehicle_speed_origin_ = radar.vehicle_speed_origin; //原始ECU车速
#ifdef SAVE_GET_DATA
    fprintf(fp_radar_file,"%d ",m_ACC_ID);
#endif
    for (int i = 0; i < NUM; ++i)
    {
        delphi_detection_array[i].target_ID= radar.delphi_detection_array[i].target_ID;
        delphi_detection_array[i].x = radar.delphi_detection_array[i].x;
        delphi_detection_array[i].y = radar.delphi_detection_array[i].y;
        delphi_detection_array[i].v = radar.delphi_detection_array[i].v;
        delphi_detection_array[i].range = radar.delphi_detection_array[i].range;
        delphi_detection_array[i].angle = radar.delphi_detection_array[i].angle;
        delphi_detection_array[i].valid = radar.delphi_detection_array[i].valid;
        delphi_detection_array[i].moving = radar.delphi_detection_array[i].moving;
        delphi_detection_array[i].moving_fast = radar.delphi_detection_array[i].moving_fast;
        delphi_detection_array[i].moving_slow = radar.delphi_detection_array[i].moving_slow;
        delphi_detection_array[i].status = radar.delphi_detection_array[i].status;
        //delphi_detection_array[i].distance = sqrt(pow(delphi_detection_array[i].x, 2) + pow(delphi_detection_array[i].y, 2));//实际上这项有原始数据，未存储

#ifdef SAVE_GET_DATA
        /*---------------毫米波雷达数据保存--------------------*/

        /***********************************************************
        valid x y range  status moving moving_fast moving_slow
        ***********************************************************/
        fprintf(fp_radar_file,"%d %d %.3f %.3f %.3f %.3f %.3f ",
            radar.delphi_detection_array[i].target_ID,
            radar.delphi_detection_array[i].status,
            radar.delphi_detection_array[i].angle,
            radar.delphi_detection_array[i].range,
            radar.delphi_detection_array[i].x,
            radar.delphi_detection_array[i].y,
            radar.delphi_detection_array[i].v
            );

        /*---------------毫米波雷达数据保存--------------------*/
#endif
    }
#ifdef SAVE_GET_DATA
    fprintf(fp_radar_file,"\n");
#endif

}


void ObjectDetection::set_radar_Data(delphi_radar_target& radar,Vehicle_Info& _vehicle_info)
{
    m_ACC_ID=radar.ACC_Target_ID;

#ifdef SAVE_GET_DATA
    fprintf(fp_radar_file,"%d ",m_ACC_ID);
#endif

    for (int i = 0; i < NUM; ++i)
    {
        delphi_detection_array[i].target_ID= radar.delphi_detection_array[i].target_ID;
        delphi_detection_array[i].x = radar.delphi_detection_array[i].x;
        delphi_detection_array[i].y = radar.delphi_detection_array[i].y;
        delphi_detection_array[i].v = radar.delphi_detection_array[i].v;
        delphi_detection_array[i].range = radar.delphi_detection_array[i].range;
        delphi_detection_array[i].angle = radar.delphi_detection_array[i].angle;
        delphi_detection_array[i].valid = radar.delphi_detection_array[i].valid;
        delphi_detection_array[i].moving = radar.delphi_detection_array[i].moving;
        delphi_detection_array[i].moving_fast = radar.delphi_detection_array[i].moving_fast;
        delphi_detection_array[i].moving_slow = radar.delphi_detection_array[i].moving_slow;
        delphi_detection_array[i].status = radar.delphi_detection_array[i].status;

#ifdef SAVE_GET_DATA
        /*---------------毫米波雷达数据保存--------------------*/

        /***********************************************************
        valid x y range  status moving moving_fast moving_slow
        ***********************************************************/
        fprintf(fp_radar_file,"%d %d %.3f %.3f %.3f %.3f %.3f ",
            radar.delphi_detection_array[i].target_ID,
            radar.delphi_detection_array[i].status,
            radar.delphi_detection_array[i].angle,
            radar.delphi_detection_array[i].range,
            radar.delphi_detection_array[i].x,
            radar.delphi_detection_array[i].y,
            radar.delphi_detection_array[i].v
            );

        /*---------------毫米波雷达数据保存--------------------*/
#endif

        //delphi_detection_array[i].distance = sqrt(pow(delphi_detection_array[i].x, 2) + pow(delphi_detection_array[i].y, 2));//实际上这项有原始数据，未存储
    }

#ifdef SAVE_GET_DATA
    fprintf(fp_radar_file,"\n");
#endif
}


void ObjectDetection::main_function2()
{
    DrawCommonElements();
    //筛选雷达点
    for(int i = 0;i<NUM;i++)
    {
        if((delphi_detection_array[i].range>1.0))//径向距离大于1m才有效
        {
            vecObj_Temp.push_back(delphi_detection_array[i]);

        }
    }

    if(!vecObj_Temp.empty()){ //有毫米波检测目标
        show_result(vecObj_Temp);//有效目标点全部显示
    }

    ShowACCTarget(m_ACC_ID);

//    cvNamedWindow("delphi_image",CV_WINDOW_NORMAL);
//    cvShowImage("delphi_image", m_Delphi_img);
//
//    int key = cvWaitKey(10);
//    if(key == 32)
//        cvWaitKey(0);

    vecObj_Temp.clear(); //清除向量


}


void ObjectDetection::compare()
{
    static int FLAG = 15;
    static int TIME = 0; //记录处理帧数
    static bool save_flag = true;
    static int flag = 0;
    static moving_object_millimeter temp = _empty_obj;

    static int FIND_ACC_ID = 0;

    for(int i = 0;i<NUM;i++)
    {
        if(delphi_detection_array[i].status!=0)
        {
            //筛选得到由XLim范围限制的前方车辆
            vecObj_Interest.push_back(delphi_detection_array[i]);
        }
    }


    final_obj = _empty_obj; //初始化要发送的数据为默认值
    if(!vecObj_Interest.empty()) //有毫米波检测目标
    {
        //show_result(vecObj_Interest); //有效目标点全部显示，ACC目标点单独标示

        if(m_ACC_ID == 0) //没有ACC目标
        {
            final_obj.range = 0;
            final_obj.v = -35.5;
            final_obj.target_ID = 0;
        }
        else
        {
            final_obj = delphi_detection_array[m_ACC_ID-1];
        }
    }

}



void ObjectDetection::show_result(moving_object_millimeter& obj_interest) //显示ACC目标
{
    CvPoint delphi_pos;
    //横坐标等于中点,只显示纵坐标
    delphi_pos.x = persMap_Middle+ obj_interest.x*METER2PIXEL;
    delphi_pos.y = (PLANE_HEIGHT - 1) - (obj_interest.y + RADAR2CAR)*METER2PIXEL;

    //鸟瞰图中显示红色点
    cvCircle(m_Delphi_img,delphi_pos, 1, cvScalar(0, 0, 255), 6);
    //ACC target ID
    sprintf(text,"ACC: %d",obj_interest.target_ID);
    cvPutText(m_Delphi_img,text,cvPoint(160,15),&cf,cvScalar(0,0,255));//red color

}


void ObjectDetection::show_result(vector<moving_object_millimeter>& valid_obj)//显示全部有效目标
{
    CvPoint delphi_pos;
    for(vector<moving_object_millimeter>::iterator it = valid_obj.begin();it!=valid_obj.end();++it)
    {
      if(it->y>10){
        continue;
      }
//      printf("<object_delphi_radar> enter in show_result...\n");
        //毫米波检测目标点在鸟瞰图上的坐标
        delphi_pos.x = persMap_Middle + (*it).x*METER2PIXEL;
        delphi_pos.y = (PLANE_HEIGHT - 1) - ((*it).y + RADAR2CAR)*METER2PIXEL;
        //鸟瞰图中显示白色点
        cvCircle(m_Delphi_img,delphi_pos, 1, cvScalar(255,255, 255), 2);
        //显示目标径向绝对速度，验证是否可以判断静态或动态目标
        float speed_abs = vehicle_info_.vehicle_speed*cos(it->angle*toRAD) + it->v;
        sprintf(text,"%.3f",speed_abs);

        //附带显示其他属性信息
        //目标序号,v, moving
        sprintf(text,"%d %.3f",
            (*it).target_ID,(*it).v);
        cvPutText(m_Delphi_img,text,cvPoint(delphi_pos.x+2,delphi_pos.y),&cf,cvScalar(0,255,255));

        /*************绘制另一幅对比图********************/
        cvCircle(m_Delphi_img_compare,delphi_pos, 1, cvScalar(255,255, 255), 2);
        float speed_abs_compare = vehicle_speed_origin_*cos(it->angle*toRAD) + it->v;
        sprintf(text,"%d %.3f %.3f %.3f",
                    (*it).target_ID,it->angle,speed_abs_compare,vehicle_info_.vehicle_speed+it->v);
        cvPutText(m_Delphi_img_compare,text,cvPoint(delphi_pos.x+2,delphi_pos.y),&cf,cvScalar(0,255,255));
        /*************绘制另一幅对比图********************/
    }

    /*cvShowImage("test1",m_Delphi_img);
    cvWaitKey(0);*/


}

void ObjectDetection::ShowACCTarget(const int& ACC_ID)
{
  if(ACC_ID != 0)//has ACC target
  {
    CvPoint delphi_pos;
    //横坐标等于中点,只显示纵坐标
    delphi_pos.x = persMap_Middle+ delphi_detection_array[ACC_ID-1].x*METER2PIXEL;
    delphi_pos.y = (PLANE_HEIGHT - 1) - (delphi_detection_array[ACC_ID-1].y + RADAR2CAR)*METER2PIXEL;

    //鸟瞰图中显示红色点
    cvCircle(m_Delphi_img,delphi_pos, 1, cvScalar(0, 0, 255), 6);
  }
  sprintf(text,"ACC: %d",ACC_ID);
  cvPutText(m_Delphi_img,text,cvPoint(350,15),&cf,cvScalar(0,0,255));//red color

}

moving_object_millimeter ObjectDetection::getSendData()
{
    final_obj.v = final_obj.v+0.13889;
    return final_obj;
}

void ObjectDetection::DrawCommonElements()
{
  cvCopy(m_Delphi_img_bak,m_Delphi_img);//每次都要清除上一次绘制的鸟瞰图

  //雷达鸟瞰图信息绘制
  //显示其他信息
   sprintf(text,"ego vehicle speed: %.3f",vehicle_info_.vehicle_speed);//从毫米波雷达获得的车速，与实际车速可能有点不同
   cvPutText(m_Delphi_img,text,cvPoint(5,15),&cf,cvScalar(0,0,255));
   sprintf(text,"ego vehicle speed origin: %.3f",vehicle_speed_origin_);//ECU实际车速
   cvPutText(m_Delphi_img,text,cvPoint(5,25),&cf,cvScalar(0,0,255));
   cvCopy(m_Delphi_img,m_Delphi_img_compare);//基本信息复制一份给另一幅图像

}
void ObjectDetection::DisplayAll()
{
  cvNamedWindow("delphi_image",CV_WINDOW_NORMAL);
  cvShowImage("delphi_image", m_Delphi_img);
  cvNamedWindow("delphi_image_compare",CV_WINDOW_NORMAL);
  cvShowImage("delphi_image_compare", m_Delphi_img_compare);
  cvWaitKey(10);
}


