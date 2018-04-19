#include <dlfcn.h>
#include "AnalysisECU.h"

CAnalysisECU::CAnalysisECU()
{

}

CAnalysisECU::~CAnalysisECU()
{
	//	data_backup.close();
}

bool  CAnalysisECU::Init(VehicleName car_type,int port)
{
	vehicle_name_ = car_type;
	/* create a UDP socket */
	if((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
	{
		perror("cannot create ecu socket\n");
		return false;
	}
	//printf("socket fd=%d\n",fd);

	/* bind the socket to any valid IP address and a specific port */
	memset((char*)&myaddr, 0, sizeof(myaddr));
	myaddr.sin_family = AF_INET;
	//myaddr.sin_addr.s_addr =inet_addr("192.168.0.254");
	myaddr.sin_addr.s_addr =htonl(INADDR_ANY);
	//myaddr.sin_addr.s_addr =htons("192.168.0.254");
	myaddr.sin_port = htons(port);
	//enable address reuse
	int ret,on=1;
	ret = setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&on,sizeof(on));

	if(bind(fd, (sockaddr *)&myaddr, sizeof(myaddr)) < 0)
	{
		perror("ECU input  bind failed");
		close(fd);
		return false;
	}

	addrlen = sizeof(myaddr); /* length of addresses */

	//霍钊添加
	//CANFound = -1;
	return true;
}

void CAnalysisECU::Update()
{
	printf("enter in Update\n");
	//  printf("waiting on port %d\n", ECU_IN_PORT);
	recvlen = recvfrom(fd, buf, BUFSIZE,0, (struct sockaddr *)&remaddr, &addrlen);
	if(recvlen>0)   ECU_DataProcFromVehicle(buf);
	recvlen = recvfrom(fd, buf, BUFSIZE,MSG_DONTWAIT, (struct sockaddr *)&remaddr, &addrlen);//20180115
	if(recvlen>0)   ECU_DataProcFromVehicle(buf);
	recvlen = recvfrom(fd, buf, BUFSIZE,MSG_DONTWAIT, (struct sockaddr *)&remaddr, &addrlen);//20180115
	if(recvlen>0)   ECU_DataProcFromVehicle(buf);
	recvlen = recvfrom(fd, buf, BUFSIZE,MSG_DONTWAIT, (struct sockaddr *)&remaddr, &addrlen);//20180115
	if(recvlen>0)   ECU_DataProcFromVehicle(buf);
	recvlen = recvfrom(fd, buf, BUFSIZE,MSG_DONTWAIT, (struct sockaddr *)&remaddr, &addrlen);//20180115
	if(recvlen>0)   ECU_DataProcFromVehicle(buf);
	recvlen = recvfrom(fd, buf, BUFSIZE,MSG_DONTWAIT, (struct sockaddr *)&remaddr, &addrlen);//20180115

	if(recvlen <= 0)
		perror("Error: ");
	else
		//printf(">>>>>%d\n",recvlen);  
		ECU_DataProcFromVehicle(buf);    //q1：recvfrom()返回读入的字节数，字节数大于0就启动ECU_DataProcFromVehicle，合适吗？


}

double CAnalysisECU::convert_ctrlvalue2steeringangle(int ctrlvalue, bool direction, double steeringratio_l, double steeringratio_r)
{
	double steeringangle=0.0;

	//double BYD_Steering_ratio_L = 2698 / 13.96 ;
	//double BYD_Steering_ratio_R = 2545 / 13.25 ;
	steeringangle = (double)(ctrlvalue - 7900) ;
	//printf("steeringangle=%f/n",steeringangle);
	if(!direction)
		steeringangle = (double)steeringangle / steeringratio_l;
	else	//turn right
		steeringangle = (double)-steeringangle / steeringratio_r;
	return steeringangle;
}

void CAnalysisECU::ECU_DataProcFromVehicle(unsigned char* data)
{
	printf("enter in Update\n");
	switch(vehicle_name_)
	{
	//receive ECU data from HUACHEN
	case CAnalysisECU::HUACHEN:
	{
		//printf("%02X",data[0]);
		//printf("%02X",data[1]);
		unsigned char static_ucECURXDataChecksum = 0;
		unsigned char datacopy[50];
		memcpy(datacopy, data, 50);

		unsigned char m_SteerMode;
		unsigned char BrakePedal;
		unsigned char tmp1;
		unsigned char checksum=0x00;
		data[0]=datacopy[48];
		data[1]=datacopy[49];

		//printf("%02X",data[0]);
		//printf("%02X",data[1]);
		for(int i=0; i < 48; i++)
		{
			data[2+i]=datacopy[i];
		}

		//check
		if((0xAA == data[0] )&&( 0x55 == data[1]))
		{
			//printf("----------%02X",data[3]);
			for(int i=0;i<49;i++)
				checksum=checksum+data[i];

			if(checksum == data[49])
			{
				//Steering wheel
				int tmp ;
				if(data[3] < 128)	  //signed exchange to unsigned
					tmp =7800 -(data[2]+(data[3]&0x07F)*256);
				else
					tmp =(32768-data[2]-(data[3]&0x07F)*256) + 7800 ;

				if(((data[6]>>7)&0x01) == 0)
				{
					//mainDlg->m_SteerMode = 0 ;
					m_SteerMode = 0 ;
					ECUData_struct.lateralctrl_enabled =  false;
				}
				else
					//mainDlg->m_SteerMode = 1 ;
					m_SteerMode = 1 ;
				ECUData_struct.lateralctrl_enabled =  true;

				tmp1 = data[15]*0.3921;
				if (tmp1 == 0)
					//mainDlg->BrakePedal = 0;		//Brake pedal depressed
					BrakePedal = 0;
				else
					//mainDlg->BrakePedal = 1;		//Brake pedal isn't depressed
					BrakePedal = 1;

				ECUData_struct.fFLRWheelAverAngle = convert_ctrlvalue2steeringangle( tmp , 0 , steeringratio_l, steeringratio_r);	  //左负右正
				//printf("steeringratio_l=%f\n",steeringratio_l);


				double PH=( data[32]+(data[33]&0x1f)*256 )*0.05625 ;
				ECUData_struct.fForwardVel = (double)PH / 3.6;	  //转换为m/s
				//printf("fFLRWheelAverAngle=%f\n",ECUData_struct.fFLRWheelAverAngle);
				//Gear
				ECUData_struct.f_shift=  (data[22]>>3)&0x0f;//档位
				ECUData_struct.f_shift1=  0;//具体档位暂无。

				//printf("f_shift=%d\n",ECUData_struct.f_shift);
				//printf("f_shift1=%d\n",data[22]);

				ECUData_struct.longitutdectrl_enabled = true;//纵向控制使能
				//ECUData_struct.lateralctrl_enabled =true;//横向控制使能

				ECUData_struct.petral_pressure = data[37];//制动压力,bar

				char a=data[16];
				char b=a>>6;

				//Brake pedal signal
				ECUData_struct.brake_enabled = ((data[16]>>6) & 0x01==0)? true :false;// : ;
				ECUData_struct.pressure_back = 0;//后油路压力,暂无，保留为0
			}
		}

		break;
	} //end VehicleName::HUACHEN

	//receive ECU data from BYD Tang
	case CAnalysisECU::BYD_TANG:
	{
		printf("recvlen is %d\n",recvlen);
		if(recvlen == 50)
		{
			unsigned char static_ucECURXDataChecksum = 0;
			unsigned char datacopy[50];
			memcpy(datacopy, data, 50);
			unsigned char tmp1,tmp2;
			data[0]=datacopy[48];
			data[1]=datacopy[49];

			for(int i=0; i < 48; i++)
			{
				data[2+i]=datacopy[i];

			}

			if( (0xAA == data[0] )&&( 0x55 == data[1]))
			{
				//方向盘角度
				int tmp=((data[3]&0xFC)/4+data[4]*64+(data[5]&0x03)*4096);

				ECUData_struct.fFLRWheelAverAngle = -convert_ctrlvalue2steeringangle( tmp , 0 , steeringratio_l, steeringratio_r);

				tmp1=data[13];//车速
				tmp2=data[14]&0x0F;
				double PH=0.06875*(tmp2*256+tmp1);
				ECUData_struct.fForwardVel = (double)PH / 3.6;

				//档位
				ECUData_struct.f_shift=  (data[2]&0x3C)/4;//档位
				ECUData_struct.f_shift1=  0;//具体档位暂无。

				ECUData_struct.pressure_back= data[20];//left 制动压力
				ECUData_struct.petral_pressure = data[21];//Right
			}

		}
	}//end VehicleName::BYD_TANG

	}


}


