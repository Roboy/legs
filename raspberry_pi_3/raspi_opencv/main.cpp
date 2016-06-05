#include <ctime>
#include <iostream>
#include <raspicam/raspicam_cv.h>
#include <opencv/cv.h>
using namespace std; 
 
int main ( int argc,char **argv ) {
   
    time_t timer_begin,timer_end;
    raspicam::RaspiCam_Cv Camera;
    cv::Mat image;
    //set camera params
    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC1 );
    //Open camera
    cout<<"Opening Camera..."<<endl;
    if (!Camera.open()) {cerr<<"Error opening the camera"<<endl;return -1;}
    //Start capture
    cout<<"Capturing ..."<<endl;
    while ( true ) {
        Camera.grab();
        Camera.retrieve ( image);
	cv::imshow("raspi camera", image);
	cv::waitKey(1);
    }
    cout<<"Stop camera..."<<endl;
    Camera.release();
}
