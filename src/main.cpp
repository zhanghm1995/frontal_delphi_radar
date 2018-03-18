#include <iostream>
#include "frontal_delphi_radar.h"
#include "object_detection_radar.h"
using namespace std;
int main(){
    FrontalDelphiRadar frontal_delphi;
    ObjectDetection object_detection_;
    object_detection_.draw_basic_info();
    bool flag = frontal_delphi.Init();
    if(flag == true){
        cout<<"hello world!"<<endl;
    }
    else{
        cout<<"error"<<endl;
        return -1;
    }
    while(1){
    frontal_delphi.Update();
    delphi_radar_target radar_data=frontal_delphi.radar_target_data();
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
