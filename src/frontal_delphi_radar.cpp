#include "frontal_delphi_radar.h"
#include <arpa/inet.h>
#include <algorithm>
#include <math.h>

#ifdef HUACHEN
const char* FRONTAL_RADAR_IP = "192.168.0.178";
#endif

#ifdef TOYOTA
const char* FRONTAL_RADAR_IP = "192.168.0.12";
#endif

using std::vector;
#define PI 3.1415926535898
const double toRAD = PI/180.0;
FrontalDelphiRadar::FrontalDelphiRadar(){
  memset((char*)&myaddr_,0,sizeof(myaddr_));
  memset(&remaddr_,0,sizeof(remaddr_));
  remaddr_.sin_addr.s_addr = inet_addr(FRONTAL_RADAR_IP);//IP
  remaddr_.sin_family = AF_INET;
  remaddr_.sin_port = htons(FRONTAL_RADAR_PORT); //port

  //initialize the parsing parameters
  radar_target_CAN_ID_vec_.resize(64);
  for(vector<unsigned int>::iterator it=radar_target_CAN_ID_vec_.begin();it!=radar_target_CAN_ID_vec_.end();++it){
    int count = 0x500+(it-radar_target_CAN_ID_vec_.begin());
    *it = count;
  }

  //reset the radar struct
  memset(&radar_target_data_,0,sizeof(radar_target_data_));
  memset(radar_data_buf_,0,sizeof(radar_data_buf_));//reset buffer

}

FrontalDelphiRadar::~FrontalDelphiRadar(){

}
bool FrontalDelphiRadar::Init(){
  //1)create radar socket
  if((radar_socket_=socket(AF_INET,SOCK_DGRAM,0))<0){
    perror("[ERROR]cannot create radar socket!");
    return false;
  }
  //2)build connecion address
  myaddr_.sin_family=AF_INET;
  myaddr_.sin_port = htons(FRONTAL_RADAR_LISTEN_PORT);
  myaddr_.sin_addr.s_addr = htonl(INADDR_ANY);
  //enable address reuse
  int ret,on;
  ret = setsockopt(radar_socket_,SOL_SOCKET,SO_REUSEADDR,&on,sizeof(on));

  //3)bind socket to specific address
  if(bind(radar_socket_,(sockaddr*)&myaddr_,sizeof(myaddr_))<0){
    perror("[ERROR]cannot bind the radar socket!");
    close(radar_socket_);
    return false;
  }


  return true;
}

bool FrontalDelphiRadar::Update(){
  //send initial data to radar
  //Send_Triggle_Signal();
  //Send vehicle info
  Send_Vehicle_Info();
  //receive radar socket data
  remaddrlen_ = sizeof(remaddr_);
  memset(radar_data_buf_,0,sizeof(radar_data_buf_));
  recv_len_ = recvfrom(radar_socket_,radar_data_buf_,RADAR_DATA_BUFFER,0,(struct sockaddr*)&remaddr_,&remaddrlen_);
  //printf("recv length is %d \n",recv_len_);
  //printf("remote address is %s and %d\n",inet_ntoa(remaddr_.sin_addr),ntohs(remaddr_.sin_port));
  if(recv_len_<0){
    perror("[ERROR]cannot receive radar data!");
    close(radar_socket_);
    return false;
  }
  else
  {
    Proc_Radar_Data();
  }
  return true;
}

bool FrontalDelphiRadar::Send_Triggle_Signal(){
  int static count = 0;
  ++count;
  if(count == 20)//发送20次触发信号
  {
    return true;
  }
  //1)specify remote address
  sockaddr_in send_addr;
  memset(&send_addr,0,sizeof(send_addr));
  send_addr.sin_addr.s_addr = inet_addr(FRONTAL_RADAR_IP);//IP
  send_addr.sin_family = AF_INET;
  send_addr.sin_port = htons(FRONTAL_RADAR_PORT); //port
  unsigned char FI=0b00001000;
  unsigned char tmpCanID1=0b00000000;
  unsigned char tmpCanID2=0b00000000;
  unsigned char tmpCanID3=0b00000100;
  unsigned char tmpCanID4=0b11110001;
  unsigned char tmpNum=0b00000000;
  //  quint8 tmpNum2 = 0b10111111;
  unsigned char tmpNum2 = 0xbf;

  //2)fill send buffer
  //send data buffer
  unsigned char send_buf[13];
  /*ID = 0X04f1 DATA = 00 00 00 00 00 00 BF 00*/
  send_buf[0]=FI;
  send_buf[1]=tmpCanID1;
  send_buf[2]=tmpCanID2;
  send_buf[3]=tmpCanID3;
  send_buf[4]=tmpCanID4;
  send_buf[5]=tmpNum;
  send_buf[6]=tmpNum;
  send_buf[7]=tmpNum;
  send_buf[8]=tmpNum;
  send_buf[9]=tmpNum;
  send_buf[10]=tmpNum;
  send_buf[11]=tmpNum2;
  send_buf[12]=tmpNum;
  int send_len = sendto(radar_socket_,send_buf,sizeof(send_buf),0,(sockaddr*)&remaddr_,sizeof(remaddr_));
  if(send_len<0){
    perror("[ERROR]cannot send initializiton data!");
    return false;
  }
  return true;
}

bool FrontalDelphiRadar::Send_Vehicle_Info(){
  //vehicle info value assignment
   //车速
   int speed_can = (int)(self_vehicle_info_.vehicle_speed/0.0625f+0.5f);
   //方向盘转角
   float steer_phi = self_vehicle_info_.steering_angle;
   unsigned char steersign = steer_phi>0?1:0;
   short steer_can = (short)abs(steer_phi);
   unsigned char bfsign = 0; //默认为0即可
   //横摆角速度
   float yawrate_phi = self_vehicle_info_.yaw_rate; //0.2用来调整偏差，根据实际情况设定
   if(yawrate_phi<-128)
   {
     yawrate_phi = -128;
   }
   else if(yawrate_phi>127.9375)
   {
     yawrate_phi = 127;
   }
   short yawrate_can = (short)(yawrate_phi/0.0625f+0.5f);
   //转弯半径
   int radius;
   if(abs(yawrate_phi/180.0f*PI)<1.0f/8191)
   {
     if(yawrate_phi<0) radius = -8192;
     else              radius = 8192;
   }
   else{
     radius = (int)(1.0f/(yawrate_phi/180.0f*PI)+0.5f);
   }

   //Send info to ID 0x4F0
   unsigned char FI=0b00001000;
   unsigned char tmpCanID1=0b00000000;
   unsigned char tmpCanID2=0b00000000;
   unsigned char tmpCanID3=0b00000100; //04
   unsigned char tmpCanID4=0b11110000; //F0
   unsigned char send_buf_4F0[13];
   memset(send_buf_4F0,0,sizeof(send_buf_4F0));
   send_buf_4F0[0]=FI;
   send_buf_4F0[1]=tmpCanID1;
   send_buf_4F0[2]=tmpCanID2;
   send_buf_4F0[3]=tmpCanID3;
   send_buf_4F0[4]=tmpCanID4;
   send_buf_4F0[5]=(speed_can>>3); //车速, m/s
   send_buf_4F0[6]=(((speed_can&0x07)<<5)|((yawrate_can>>8)&0x0F)|(bfsign<<4));//横摆角速度，行驶方向
   send_buf_4F0[7]=((yawrate_can)&0xFF);//横摆角速度
   send_buf_4F0[8]=(0x80|(radius>>8));//横摆角速度有效位,转弯半径
   send_buf_4F0[9]=(radius&0xFF);//转弯半径
   send_buf_4F0[10]=(0x00|(steersign<<6)|(steer_can>>5));//方向盘转角有效位，方向盘转角方向，方向盘转角
   send_buf_4F0[11]=((steer_can&0x1F)<<3);
   //发送
   int send_len = sendto(radar_socket_,send_buf_4F0,sizeof(send_buf_4F0),0,(sockaddr*)&remaddr_,sizeof(remaddr_));
   if(send_len<0){
     perror("[ERROR]cannot send send_buf_4F0 data!");
   }


   //Send info to ID 0x4F1
   unsigned char send_buf_4F1[13];
   memset(send_buf_4F1,0,sizeof(send_buf_4F1));
   send_buf_4F1[0]=0b00001000;
   send_buf_4F1[1]=0x00;
   send_buf_4F1[2]=0x00;
   send_buf_4F1[3]=0x04;
   send_buf_4F1[4]=0xF1;
   send_buf_4F1[10]= 0;//横向安装偏差为0,
   send_buf_4F1[11]=(1<<7)|(1<<6)|63;//雷达辐射命令位置1，阻塞关闭位置1，最大跟踪目标数64
   send_buf_4F1[12] =(1<<5);//速度有效位置
   //发送
   send_len = sendto(radar_socket_,send_buf_4F1,sizeof(send_buf_4F1),0,(sockaddr*)&remaddr_,sizeof(remaddr_));
   if(send_len<0){
     perror("[ERROR]cannot send send_buf_4F1 data!");
   }
   //Send info to ID 0x5F2
   unsigned char send_buf_5F2[13];
   memset(send_buf_5F2,0,sizeof(send_buf_5F2));
   send_buf_5F2[0] = 0b00001000;
   send_buf_5F2[7] = (10>>1);//长距离模式的角度为10度
   send_buf_5F2[8] = ((10&0x01)<<7)|45;//短距离模式的角度是45度
   send_buf_5F2[9] = 65; //雷达的安装高度为45cm
   //发送
   send_len = sendto(radar_socket_,send_buf_5F2,sizeof(send_buf_5F2),0,(sockaddr*)&remaddr_,sizeof(remaddr_));
   if(send_len<0){
     perror("[ERROR]cannot send send_buf_5F2 data!");
   }

   return true;


}
void FrontalDelphiRadar::Proc_Radar_Data(){
  int can_frame_count = recv_len_/13;//each can frame contains 13 bytes
//  printf("recv_len_ is %d ========can_frame_count is %d \n",recv_len_,can_frame_count);
  for(int i=0;i<can_frame_count;++i){//a udp data frame may contain numbers of CAN frames
    unsigned char* buf = &(radar_data_buf_[i*13]);
    unsigned int tmpCanID = 0;
    unsigned char tmpdata[8] = {0};
    tmpCanID=buf[1]<<24|buf[2]<<16|buf[3]<<8|buf[4];
    tmpdata[0]=buf[5];tmpdata[1]=buf[6];tmpdata[2]=buf[7];
    tmpdata[3]=buf[8];tmpdata[4]=buf[9];tmpdata[5]=buf[10];
    tmpdata[6]=buf[11];tmpdata[7]=buf[12];

    /*******************************/
    /*parsing the radar data we want*/
    /*******************************/
    //get the most dangerous target ID
    unsigned short TrackID_1 = 0,TrackID_2 = 0;
    if(tmpCanID == 0x4E3){
      TrackID_1 = tmpdata[1];//动态ACC目标
      TrackID_2 = tmpdata[7];//静态ACC目标
    }
    radar_target_data_.ACC_Target_ID = TrackID_1; //choose dynamic ACC target first
    if(TrackID_1 == 0){
      radar_target_data_.ACC_Target_ID = TrackID_2;
    }
    //get the target CAN ID data
    vector<unsigned int>::iterator iter = find(radar_target_CAN_ID_vec_.begin(),radar_target_CAN_ID_vec_.end(),tmpCanID);
    if(iter!=radar_target_CAN_ID_vec_.end()){ //obtain valid radar target data
      int m = iter-radar_target_CAN_ID_vec_.begin();//the m-th target, m = 0~63
      radar_target_data_.delphi_detection_array[m].target_ID = m+1; //target_ID = 1~64
      //target status
      unsigned short temp_S1 = tmpdata[1];
      unsigned short temp_S2 = temp_S1&0x00E0;
      radar_target_data_.delphi_detection_array[m].status = temp_S2>>5;
      printf("mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm is %d \n",m);
      unsigned short temp_D1;
      unsigned short temp_D2;
      unsigned short temp_V1;
      unsigned short temp_V2;
      unsigned short temp_V3;
      //range  Unit: m
      temp_D1 = tmpdata[2];
      temp_D2 = tmpdata[3];
      radar_target_data_.delphi_detection_array[m].range = ((temp_D2)|((temp_D1&0x0007)<<8))*0.1f;
      printf("range is %f \n",radar_target_data_.delphi_detection_array[m].range);

      //range_rate  Unit: m/s
      temp_V1 = tmpdata[6];
      temp_V2 = tmpdata[7];
      unsigned short  temp_V=temp_V1&0x0020;
      if (temp_V==0)
      {
        radar_target_data_.delphi_detection_array[m].v =(temp_V2|((temp_V1&0x003F)<<8))*0.01f;//Unit: m/s
      }
      if (temp_V==0x0020)
      {
        unsigned short temp_0=((temp_V1&0x003F) <<8)|temp_V2;
        unsigned short temp_1=temp_0 - 1;
        unsigned short temp_2=(~temp_1) & 0x1FFF;
        radar_target_data_.delphi_detection_array[m].v=-(temp_2*0.01f);//Unit: m/s
      }
      printf("range rate is %f \n",radar_target_data_.delphi_detection_array[m].v);

      //angle  Unit:degree
      unsigned short temp_A1=tmpdata[1];
      unsigned short temp_A2=tmpdata[2];
      unsigned short temp_A3=temp_A1&0x0010;
      if (temp_A3==0)
      {
        radar_target_data_.delphi_detection_array[m].angle=(((temp_A1&0x000F)<<5)|((temp_A2&0x00F8)>>3))*0.1f;
      }
      if(temp_A3==0x0010)
      {
        unsigned short temp_3=((temp_A1&0x000F)<<5)|((temp_A2&0x00F8)>>3);
        unsigned short temp_4=temp_3 - 1;
        unsigned short temp_5=(~temp_4) & 0x01FF;
        radar_target_data_.delphi_detection_array[m].angle=-temp_5*0.1f;
      }
      printf("angle is %f \n",radar_target_data_.delphi_detection_array[m].angle);

      //calculate x,y   Unit: m
      radar_target_data_.delphi_detection_array[m].x=radar_target_data_.delphi_detection_array[m].range*sin(radar_target_data_.delphi_detection_array[m].angle*toRAD);
      radar_target_data_.delphi_detection_array[m].y=radar_target_data_.delphi_detection_array[m].range*cos(radar_target_data_.delphi_detection_array[m].angle*toRAD);
    }//end if(iter!=radar_target_CAN_ID_vec_.end())

#if 1 //解析一些运动属性
    if(tmpCanID==0x540)
    {
      ////先解析Group_ID
      unsigned short temp_A1 = tmpdata[0];
      unsigned short temp_A2 = temp_A1&0x000F; //取出后四位
      switch(temp_A2)
      {
      case 0://第0组
        for(int j = 0;j<7;++j)
        {
          //解析Moving状态
          unsigned short temp_D1 = (tmpdata[j+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;
          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;

        }
        break;
      case 1://第1组
        for(int j = 7;j<14;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-7+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j-7+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-7+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 2://第2组
        for(int j = 14;j<21;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-14+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j-14+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-14+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 3://第3组
        for(int j = 21;j<28;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-21+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j-21+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-21+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 4://第4组
        for(int j = 28;j<35;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-28+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j-28+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-28+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 5://第5组
        for(int j = 35;j<42;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-35+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j-35+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-35+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 6://第6组
        for(int j = 42;j<49;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-42+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j-42+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-42+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 7://第7组
        for(int j = 49;j<56;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-49+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j-49+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-49+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 8://第8组
        for(int j = 56;j<63;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-56+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short  temp_S1 = (tmpdata[j-56+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-56+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 9://第9组
        unsigned short temp_D1 = (tmpdata[1])&0x0020;
        unsigned short temp_D2 = temp_D1>>5;
        radar_target_data_.delphi_detection_array[63].moving=temp_D2;

        //解析Movable_fast状态
        unsigned short temp_S1 = (tmpdata[1])&0x0080;
        unsigned short temp_S2 = temp_S1>>7;
        radar_target_data_.delphi_detection_array[63].moving_fast=temp_S2;
        //解析Movable_slow状态
        temp_S1 = (tmpdata[1])&0x0040;
        temp_S2 = temp_S1>>6;
        radar_target_data_.delphi_detection_array[63].moving_slow=temp_S2;
        break;
      }//end switch
    }//end if(tmpCanID==0x540)
#endif

  }

}
void FrontalDelphiRadar::Parse_Radar_Data()
{
  int can_frame_count = recv_len_/13;//each can frame contains 13 bytes
 //  printf("recv_len_ is %d ========can_frame_count is %d \n",recv_len_,can_frame_count);
   for(int i=0;i<can_frame_count;++i){//a udp data frame may contain numbers of CAN frames
     unsigned char* buf = &(radar_data_buf_[i*13]);
     unsigned int tmpCanID = 0;
     unsigned char tmpdata[8] = {0};
     tmpCanID=radar_data_buf_[1]<<24|radar_data_buf_[2]<<16|radar_data_buf_[3]<<8|radar_data_buf_[4];
     tmpdata[0]=radar_data_buf_[5];tmpdata[1]=radar_data_buf_[6];tmpdata[2]=radar_data_buf_[7];
     tmpdata[3]=radar_data_buf_[8];tmpdata[4]=radar_data_buf_[9];tmpdata[5]=radar_data_buf_[10];
     tmpdata[6]=radar_data_buf_[11];tmpdata[7]=radar_data_buf_[12];

     /*******************************/
     /*parsing the radar data we want*/
     /*******************************/
     //get the most dangerous target ID
     unsigned short TrackID_1 = 0,TrackID_2 = 0;//important!! must initialize!!!
     if(tmpCanID == 0x4E3){
       TrackID_1 = tmpdata[1];//动态ACC目标
       TrackID_2 = tmpdata[7];//静态ACC目标
     }
     radar_target_data_.ACC_Target_ID = TrackID_1; //choose dynamic ACC target first
     if(TrackID_1 == 0){
       radar_target_data_.ACC_Target_ID = TrackID_2;
     }
     //get the target CAN ID data
     static int m = 0;
     if(tmpCanID==0x500+m)
     {
       radar_target_data_.delphi_detection_array[m].target_ID = m+1; //target_ID = 1~64
       //target status
       unsigned short temp_S1 = tmpdata[1];
       unsigned short temp_S2 = temp_S1&0x00E0;
       radar_target_data_.delphi_detection_array[m].status = temp_S2>>5;
       printf("mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm is %d \n",m);
       unsigned short temp_D1;
       unsigned short temp_D2;
       unsigned short temp_V1;
       unsigned short temp_V2;
       unsigned short temp_V3;
       //range  Unit: m
       temp_D1 =tmpdata[2];
       temp_D2 = tmpdata[3];
       radar_target_data_.delphi_detection_array[m].range = ((temp_D2)|((temp_D1&0x0007)<<8))*0.1f;
       printf("range is %f \n",radar_target_data_.delphi_detection_array[m].range);

       //range_rate  Unit: m/s
       temp_V1 = tmpdata[6];
       temp_V2 = tmpdata[7];
       unsigned short  temp_V=temp_V1&0x0020;
       if (temp_V==0)
       {
         radar_target_data_.delphi_detection_array[m].v =(temp_V2|((temp_V1&0x003F)<<8))*0.01f;//Unit: m/s
       }
       if (temp_V==0x0020)
       {
         unsigned short temp_0=((temp_V1&0x003F) <<8)|temp_V2;
         unsigned short temp_1=temp_0 - 1;
         unsigned short temp_2=(~temp_1) & 0x1FFF;
         radar_target_data_.delphi_detection_array[m].v=-(temp_2*0.01f);//Unit: m/s
       }
       printf("range rate is %f \n",radar_target_data_.delphi_detection_array[m].v);

       //angle  Unit:degree
       unsigned short temp_A1=tmpdata[1];
       unsigned short temp_A2=tmpdata[2];
       unsigned short temp_A3=temp_A1&0x0010;
       if (temp_A3==0)
       {
         radar_target_data_.delphi_detection_array[m].angle=(((temp_A1&0x000F)<<5)|((temp_A2&0x00F8)>>3))*0.1f;
       }
       if(temp_A3==0x0010)
       {
         unsigned short temp_3=((temp_A1&0x000F)<<5)|((temp_A2&0x00F8)>>3);
         unsigned short temp_4=temp_3 - 1;
         unsigned short temp_5=(~temp_4) & 0x01FF;
         radar_target_data_.delphi_detection_array[m].angle=-temp_5*0.1f;
       }
       printf("angle is %f \n",radar_target_data_.delphi_detection_array[m].angle);

       //calculate x,y   Unit: m
       radar_target_data_.delphi_detection_array[m].x=radar_target_data_.delphi_detection_array[m].range*sin(radar_target_data_.delphi_detection_array[m].angle*toRAD);
       radar_target_data_.delphi_detection_array[m].y=radar_target_data_.delphi_detection_array[m].range*cos(radar_target_data_.delphi_detection_array[m].angle*toRAD);

       ++m;
       if(m==64)
       {
         m = 0;
       }
     }

#if 1 //解析一些运动属性
    if(tmpCanID==0x540)
    {
      ////先解析Group_ID
      unsigned short temp_A1 = tmpdata[0];
      unsigned short temp_A2 = temp_A1&0x000F; //取出后四位
      switch(temp_A2)
      {
      case 0://第0组
        for(int j = 0;j<7;++j)
        {
          //解析Moving状态
          unsigned short temp_D1 = (tmpdata[j+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;
          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;

        }
        break;
      case 1://第1组
        for(int j = 7;j<14;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-7+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j-7+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-7+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 2://第2组
        for(int j = 14;j<21;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-14+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j-14+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-14+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 3://第3组
        for(int j = 21;j<28;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-21+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j-21+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-21+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 4://第4组
        for(int j = 28;j<35;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-28+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j-28+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-28+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 5://第5组
        for(int j = 35;j<42;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-35+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j-35+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-35+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 6://第6组
        for(int j = 42;j<49;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-42+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j-42+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-42+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 7://第7组
        for(int j = 49;j<56;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-49+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short temp_S1 = (tmpdata[j-49+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-49+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 8://第8组
        for(int j = 56;j<63;++j)
        {
          unsigned short temp_D1 = (tmpdata[j-56+1])&0x0020;
          unsigned short temp_D2 = temp_D1>>5;
          radar_target_data_.delphi_detection_array[j].moving = temp_D2;

          //解析Movable_fast状态
          unsigned short  temp_S1 = (tmpdata[j-56+1])&0x0080;
          unsigned short temp_S2 = temp_S1>>7;
          radar_target_data_.delphi_detection_array[j].moving_fast=temp_S2;
          //解析Movable_slow状态
          temp_S1 = (tmpdata[j-56+1])&0x0040;
          temp_S2 = temp_S1>>6;
          radar_target_data_.delphi_detection_array[j].moving_slow=temp_S2;
        }
        break;
      case 9://第9组
        unsigned short temp_D1 = (tmpdata[1])&0x0020;
        unsigned short temp_D2 = temp_D1>>5;
        radar_target_data_.delphi_detection_array[63].moving=temp_D2;

        //解析Movable_fast状态
        unsigned short temp_S1 = (tmpdata[1])&0x0080;
        unsigned short temp_S2 = temp_S1>>7;
        radar_target_data_.delphi_detection_array[63].moving_fast=temp_S2;
        //解析Movable_slow状态
        temp_S1 = (tmpdata[1])&0x0040;
        temp_S2 = temp_S1>>6;
        radar_target_data_.delphi_detection_array[63].moving_slow=temp_S2;
        break;
      }//end switch
    }//end if(tmpCanID==0x540)
#endif

   }

}

delphi_radar_target FrontalDelphiRadar::radar_target_data(){
  return this->radar_target_data_;
}
Vehicle_Info FrontalDelphiRadar::vehicle_info(){
  return this->self_vehicle_info_;
}

void FrontalDelphiRadar::set_self_vehicle_info(const double& yaw_rate,const double& vehicle_speed,const double& steering_angle){
  self_vehicle_info_.yaw_rate = yaw_rate;
  self_vehicle_info_.vehicle_speed = vehicle_speed;
  self_vehicle_info_.steering_angle = steering_angle;
}
void FrontalDelphiRadar::set_self_vehicle_info(const Vehicle_Info& vehicle_info){
  self_vehicle_info_ = vehicle_info;
}

