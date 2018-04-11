/*==================================================================
 *    Function    ：receive udp data of delphi radar and parse them
 *    相关说明：
 *    作者    ：  zhanghm
 *    创建日期    ：20180317
 *    修改记录：
/*==================================================================*/
#ifndef FRONTAL_DELPHI_RADAR_H_
#define FRONTAL_DELPHI_RADAR_H_
//C/C++
#include <cstdio>
#include <string.h>
#include <iostream>
#include <string>
#include <vector>
//UDP socket
#include <dlfcn.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/in.h>
//project headers
#include "TypeDef.h" //radar and vehicle information struct

#define TOYOTA
#ifdef TOYOTA
//const definition
const int FRONTAL_RADAR_PORT = 4001;
const int FRONTAL_RADAR_LISTEN_PORT = 8101; //local listen port
#endif

#ifdef HUACHEN
//const definition
const int FRONTAL_RADAR_PORT = 4001;
const int FRONTAL_RADAR_LISTEN_PORT = 8010; //local listen port
#endif

const short RADAR_DATA_BUFFER = 650; //buffer 1000 bytes

class FrontalDelphiRadar{
public:

public:
  //constructors
  FrontalDelphiRadar();
  ~FrontalDelphiRadar();

  bool Init();//init socket and bind socket
  bool Update();//update socket to receive data, should be called in a loop
  delphi_radar_target radar_target_data(); //get radar target data
  Vehicle_Info vehicle_info(); //get vehicle information
  //get self vehicle information
  void set_self_vehicle_info(const double& yaw_rate,const double& vehicle_speed,const double& steering_angle=0); //set self_vehicle_info member
  void set_self_vehicle_info(const Vehicle_Info& vehicle_info);
private:
  void Proc_Radar_Data();//important! parse radar data
  void Parse_Radar_Data();
  bool Send_Vehicle_Info();//send vehicle information to radar
  bool Send_Triggle_Signal();//send triggle signal to radar to get normal data

private:
  //udp socket
  int radar_socket_; //socket used to communicate with radar,send or receive data from radar
  sockaddr_in myaddr_; //listen port info
  sockaddr_in remaddr_; //remote address
  socklen_t remaddrlen_;
  int recv_len_; //recvfrom function return, indicate receive how many bytes in a UDP package
  //address struct for send vehicle info

  //radar data parsing
  unsigned char radar_data_buf_[RADAR_DATA_BUFFER];
  std::vector<unsigned int> radar_target_CAN_ID_vec_;
  delphi_radar_target radar_target_data_;//final output
  //send vehicle info
  Vehicle_Info self_vehicle_info_;






};

#endif /*FRONTAL_DELPHI_RADAR_H_*/
