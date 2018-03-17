#include "frontal_delphi_radar.h"
#include <arpa/inet.h>
#include <algorithm>
#include <math.h>

const char* FRONTAL_RADAR_IP = "192.168.0.178";
using std::vector;
#define PI 3.1415926535898
const double toRAD = PI/180.0;
FrontalDelphiRadar::FrontalDelphiRadar(){
  memset((char*)&myaddr_,0,sizeof(myaddr_));
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
  Send_Triggle_Signal();

  //receive radar socket data
  remaddrlen_ = sizeof(remaddr_);
  recv_len_ = recvfrom(radar_socket_,radar_data_buf_,RADAR_DATA_BUFFER,0,(struct sockaddr*)&remaddr_,&remaddrlen_);
  //printf("recv length is %d \n",recv_len_);
  //printf("remote address is %s and %d\n",inet_ntoa(remaddr_.sin_addr),ntohs(remaddr_.sin_port));
  if(recv_len_<0){
    perror("[ERROR]cannot receive radar data!");
    close(radar_socket_);
    return false;
  }
  else
    Proc_Radar_Data();
  //usleep(50000);
}

bool FrontalDelphiRadar::Send_Triggle_Signal(){
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
  //TODO: vehicle info value assignment
  int speed_can = 0;
  short yawrate_can = 0;
  int radius;
  unsigned char steersign;
  short steer_can = 0;
  unsigned char bfsign = 0;

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
  send_buf_4F0[5]=(speed_can>>3); //host vehicle spedd, m/s
  send_buf_4F0[6]=(((speed_can&0x07)<<5)|((yawrate_can>>8)&0x0F)|(bfsign<<4));
  send_buf_4F0[7]=((yawrate_can)&0xFF);
  send_buf_4F0[8]=(0x80|(radius>>8));//横摆角速度有效位,转弯半径
  send_buf_4F0[9]=(radius&0xFF);
  send_buf_4F0[10]=(0x80|(steersign<<6)|(steer_can>>5));
  send_buf_4F0[11]=((steer_can&0x1F)<<3);


  //Send info to ID 0x4F1
  unsigned char send_buf_4F1[13];
  memset(send_buf_4F1,0,sizeof(send_buf_4F1));
  send_buf_4F1[0]=0b00001000;
  send_buf_4F1[1]=0x00;
  send_buf_4F1[2]=0x00;
  send_buf_4F1[3]=0x04;
  send_buf_4F1[4]=0xF1;
  send_buf_4F1[10]= 0;//横向安装偏差
  send_buf_4F1[11]=(1<<7)|(1<<6)|63;
  send_buf_4F1[12] =(1<<5);//速度有效位置

  //Send info to ID 0x5F2


}
void FrontalDelphiRadar::Proc_Radar_Data(){
  int can_frame_count = recv_len_/13;//each can frame contains 13 bytes
  for(int i=0;i<can_frame_count;++i){//a udp data frame may contain numbers of CAN frames
    unsigned char* buf = &(radar_data_buf_[i*13]);
    unsigned int tmpCanID;
    unsigned char tmpdata[8];
    tmpCanID=buf[1]<<24|buf[2]<<16|buf[3]<<8|buf[4];
    tmpdata[0]=buf[5];tmpdata[1]=buf[6];tmpdata[2]=buf[7];
    tmpdata[3]=buf[8];tmpdata[4]=buf[9];tmpdata[5]=buf[10];
    tmpdata[6]=buf[11];tmpdata[7]=buf[12];

    //parsing the radar data we want
    vector<unsigned int>::iterator iter = find(radar_target_CAN_ID_vec_.begin(),radar_target_CAN_ID_vec_.end(),tmpCanID);
    if(iter!=radar_target_CAN_ID_vec_.end()){ //obtain valid radar target data
      int m = iter-radar_target_CAN_ID_vec_.begin();//the m-th target
      printf("mmmmmmmmm is %d \n",m);
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

    }


  }

}


delphi_radar_target FrontalDelphiRadar::radar_target_data(){
  return radar_target_data_;
}

