#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <opencv2/objdetect.hpp>

#include <raspicam/raspicam_cv.h>

#include <iostream>

#include "pca9685.h"
#include <wiringPi.h>

using namespace cv;
using namespace std;

#define BB_VIDEO_WIDTH 		360
#define BB_VIDEO_HEIGHT 	240
#define BB_MIN_FACE		15
#define BB_MAX_FACE		150

#define BB_DO_FLIP		1

//pca9685 defines
#define PCA_PIN_BASE 300
#define PCA_MAX_PWM 4096
#define PCA_HERTZ 50

//drive servo PIN at angle ANGLE for SG90 servo: 1ms=>0deg, 1.5ms=>90deg, 2ms=>180deg
//angle: [0,180], pin: PIN_BASE(controller)+SERVO_NUM(0-15) or PIN_BASE(controller)+16 for all servos
int driveDegs(float angle, int pin)
{
  if (angle<0) angle = 0;
  if (angle>180) angle = 180;
  int ticks = (int)(PCA_MAX_PWM * (angle/180.0f + 1.0f) / 20.0f + 0.5);
  pwmWrite(pin, ticks);
}

int main( int argc, char** argv )
{    
    
  /*
    //Init camera
    raspicam::RaspiCam_Cv Camera;
    Camera.set ( CV_CAP_PROP_FORMAT, CV_8UC1 ); //grayscale
    if ( !Camera.open() ) 
    {
        cout<<"Error opening camera"<<endl;
        return 0;
    }
    cout<<"Connected to camera ="<<Camera.getId() <<endl;
    
    //Init detector
    CascadeClassifier detector;
    if(!detector.load("./lbpcascade_frontalface.xml"))
    {
	cout<<"Cannot load detector"<<endl;
	return 0;
    }
    
    vector<Rect> detections;
    cv::Mat image;
    
    for (;;) 
    { 
      try{	  
	  //get frame
	  Camera.grab();
	  Camera.retrieve ( image );
	  
	  //transform image and detect face
	  detections.clear();
	  resize(image,image,Size(BB_VIDEO_WIDTH,BB_VIDEO_HEIGHT));
#if BB_DO_FLIP
	  flip(image,image,0);
#endif
	  detector.detectMultiScale(image, detections, 1.1, 2, 0, Size(BB_MIN_FACE,BB_MIN_FACE), Size(BB_MAX_FACE,BB_MAX_FACE));
	  
	  //draw face
	  for (int i=0; i<detections.size(); i++)
	    rectangle(image, detections[i], cv::Scalar(255,0,0));
	  imshow("frame",image);
	  
	  //break if key pressed
	  if(waitKey(1)!=-1)
	  {
	    break;
	  }
	  
	}
	catch(cv::Exception e)
	{
	  cout<<e.what()<<endl;
	  break;
	}
    }
    
    Camera.release();
    
   */
  
    // Setup with pinbase 300 and i2c location 0x40 (default for pca9685)
    // PWM period for SG90 servos is 20ms (50Hz)
    // Duty cycle for SG90 servos is 1-2ms (-90 -- 90 deg); 1.5 ms is neutral (0 deg)
    int fd = pca9685Setup(PCA_PIN_BASE, 0x40, PCA_HERTZ);
    if (fd < 0)
    {
	    cout<<"Error in init PCA9685!"<<endl;
	    return fd;
    }
    // Reset all output
    pca9685PWMReset(fd);
    
    //drive all servos to 5 deg
    driveDegs(5, PCA_PIN_BASE + 16);
    delay(1000);
    
    while(true)
    {
      for (int i=0; i<5; i++)
      {
	driveDegs(175, PCA_PIN_BASE + i);
	delay(200);
      }
      for (int i=0; i<5; i++)
      {
	driveDegs(5, PCA_PIN_BASE + i);
	delay(200);
      }
    }
  
    return 0;
 
}