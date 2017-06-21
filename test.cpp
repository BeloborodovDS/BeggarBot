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

/**
 * Calculate the number of ticks the signal should be high for the required amount of time
 */
int calcTicks(float impulseMs, int hertz)
{
	float cycleMs = 1000.0f / hertz;
	return (int)(PCA_MAX_PWM * impulseMs / cycleMs + 0.5f);
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
    
    //set all to -90
    int tick = calcTicks(1.1, PCA_HERTZ);
    pwmWrite(PCA_PIN_BASE + 16, tick);
    delay(1000);
    
    while(true)
    {
      tick = calcTicks(1.9, PCA_HERTZ);
      for (int i=0; i<5; i++)
      {
	pwmWrite(PCA_PIN_BASE + i, tick);
	delay(200);
      }
      
      tick = calcTicks(1.1, PCA_HERTZ);
      for (int i=0; i<5; i++)
      {
	pwmWrite(PCA_PIN_BASE + i, tick);
	delay(200);
      }
    }
  
    return 0;
 
}