#include "object_detection_radar.h"


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
    m_Obj_Interest_img = cvCreateImage(cvSize(PLANE_WIDTH, PLANE_HEIGHT), IPL_DEPTH_8U, 3);
    cvZero(m_Delphi_img_bak);
    cvZero(m_Delphi_img);
    cvZero(m_Obj_Interest_img);

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

    sprintf(text,"Tag: number,status,range,v,moving,moving_fast,moving_slow");
    //cvPutText(m_Delphi_img_bak,text,cvPoint(1,15),&cf,CV_RGB(255,0,255));

    //cvPutText(m_Delphi_img_bak,"1090",cvPoint(340,15),&cf,CV_RGB(255,0,255));


//    cvShowImage("delphi_image", m_Delphi_img_bak);
//    cvWaitKey(50);


}

void ObjectDetection::get_radar_Data(delphi_radar_target& radar)
{

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
    }
    m_ACC_ID=radar.ACC_Target_ID;



}


void ObjectDetection::get_radar_Data(delphi_radar_target& radar,Vehicle_Info& _vehicle_info)
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
    cvCopy(m_Delphi_img_bak,m_Delphi_img);//每次都要清除上一次绘制的鸟瞰图
    //筛选雷达点
    for(int i = 0;i<NUM;i++)
    {
        if((delphi_detection_array[i].y>0.5))
        {
            vecObj_Temp.push_back(delphi_detection_array[i]);

        }
    }

    if(!vecObj_Temp.empty()){ //有毫米波检测目标
        show_result(vecObj_Temp);//有效目标点全部显示
    }

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

}


void ObjectDetection::show_result(vector<moving_object_millimeter>& valid_obj)//显示全部有效目标
{
    CvPoint delphi_pos;
    for(vector<moving_object_millimeter>::iterator it = valid_obj.begin();it!=valid_obj.end();++it)
    {
        //毫米波检测目标点在鸟瞰图上的坐标
        delphi_pos.x = persMap_Middle + (*it).x*METER2PIXEL;
        delphi_pos.y = (PLANE_HEIGHT - 1) - ((*it).y + RADAR2CAR)*METER2PIXEL;

        //鸟瞰图中显示白色点
        cvCircle(m_Delphi_img,delphi_pos, 1, cvScalar(255,255, 255), 2);

        //附带显示其他属性信息
        //目标序号,status，range,v
//        sprintf(text,"%d",
//            (*it).target_ID);
//        cvPutText(m_Delphi_img,text,cvPoint(delphi_pos.x+2,delphi_pos.y),&cf,cvScalar(0,255,255));
    }
    /*cvShowImage("test1",m_Delphi_img);
    cvWaitKey(0);*/


}

void ObjectDetection::show_result2(double& obj_interest)
{


    CvPoint delphi_pos;
    //横坐标等于中点,只显示纵坐标
    delphi_pos.x = persMap_Middle;
    delphi_pos.y = (PLANE_HEIGHT - 1) - ((int)obj_interest + RADAR2CAR)*METER2PIXEL;
    //鸟瞰图中显示红色点
    cvCircle(m_Delphi_img,delphi_pos, 1, cvScalar(0, 0, 255), 2);
}

moving_object_millimeter ObjectDetection::getSendData()
{
    final_obj.v = final_obj.v+0.13889;
    return final_obj;
}


