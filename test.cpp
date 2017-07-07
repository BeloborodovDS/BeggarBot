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

//pca9685 defines
#define PCA_PIN_BASE 		300
#define PCA_MAX_PWM 		4096 //max ticks in PWM signal
#define PCA_HERTZ 		50

//calibrated: for SG90
#define SERVO_MS_MIN 		0.6
#define SERVO_MS_MAX 		2.4
#define SERVO_MAX_SPEED		0.6 //degrees per ms - from datasheet

#define HEAD_DT			20 //ms time span 
#define HEAD_MAX_STEP		HEAD_DT * SERVO_MAX_SPEED

//PINS
#define PIN_EYEBROW 		PCA_PIN_BASE  //eyebrow servo at pin location 0 on PCA controller
#define PIN_HEAD		PCA_PIN_BASE + 1 //head servo at pin location 1 on PCA controller
#define PIN_PCA			PCA_PIN_BASE + 16 //all pins on PCA controller 

//drive servo PIN at angle ANGLE for SG90 servo
//angle: [0,180], pin: PIN_BASE(controller)+SERVO_NUM(0-15) or PIN_BASE(controller)+16 for all servos
void driveDegs(float angle, int pin)
{
  if (angle<0) angle = 0;
  if (angle>180) angle = 180;
  int ticks = (int)(PCA_MAX_PWM * (angle/180.0f*(SERVO_MS_MAX-SERVO_MS_MIN) + SERVO_MS_MIN) / 20.0f + 0.5); //20ms is pulse width (~50 hz)
  pwmWrite(pin, ticks);
}

void driveMs(float Ms, int pin)
{
  float cycleMs = 1000.0f / PCA_HERTZ;
  int ticks = (int)(PCA_MAX_PWM * Ms / cycleMs + 0.5f);
  pwmWrite(pin, ticks);
}

//move eyebrows: 
// 0 for neutral expression
// >0 for kind expression
// <0 for angry expression
void frown(int mode)
{
  if(mode==0)
    driveDegs(60, PIN_EYEBROW);
  else if (mode>0)
    driveDegs(25, PIN_EYEBROW);
  else
    driveDegs(100, PIN_EYEBROW);
  delay(1000);
}

//set initial position
void resetRobot()
{
  driveDegs(90, PIN_HEAD);
  frown(0);
  delay(2000);
}

//Drive head from angle an_from to angle an_to with specified speed
//speed is ratio from SERVO_MAX_SPEED, in [0,1]
//!!! valid an_from value must be provided
void driveHead(float an_from, float an_to, float speed = 0.1)
{
  if (speed > 1) speed = 1;
  if (speed < 0) speed = 0;
  if (an_to > 135) an_to = 135; //angle is limited in [45, 135] for safety reasons
  if (an_to < 45) an_to = 45;
  if (an_from < 0) an_from = 0;
  if (an_from > 180) an_from = 180;
  
  float step = HEAD_MAX_STEP*speed; //deg step for each time span
  
  if (an_from < an_to)  
    while(an_from < an_to) //forward
    {
      an_from += step;
      driveDegs(an_from, PIN_HEAD);
      delay(HEAD_DT);
    }
  else
    while(an_from > an_to) //backwards
    {
      an_from -= step;
      driveDegs(an_from, PIN_HEAD);
      delay(HEAD_DT);
    }
    
  //fix imprecise position
  driveDegs(an_to, PIN_HEAD);
  delay(HEAD_DT);
}


int main( int argc, char** argv )
{    
    
    //------------------------------------------------INIT-----------------------------------------------------------------
  
    //Init camera
    raspicam::RaspiCam_Cv Camera;
    Camera.set ( CV_CAP_PROP_FORMAT, CV_8UC1 ); //grayscale
    if ( !Camera.open() ) 
    {
        cout<<"Error opening camera"<<endl;
        return 0;
    }
    cout<<"Connected to camera ="<<Camera.getId() <<endl;
    
    //Init face detector
    CascadeClassifier detector;
    if(!detector.load("./lbpcascade_frontalface.xml"))
    {
	cout<<"Cannot load detector"<<endl;
	return 0;
    }
    cout<<"Face detector loaded"<<endl;
  
    // Setup PCA with pinbase 300 and i2c location 0x40 (default for pca9685)
    // PWM period for SG90 servos is 20ms (50Hz)
    int pca_fd = pca9685Setup(PCA_PIN_BASE, 0x40, PCA_HERTZ);
    if (pca_fd < 0)
    {
	cout<<"Error in init PCA9685!"<<endl;
	return 0;
    }
    // Reset all output
    pca9685PWMReset(pca_fd);
    cout<<"PCA controller connected"<<endl;
     
    resetRobot();
    cout<<"Robot reset"<<endl;
    
    //-------------------------------------------------MAIN BODY-----------------------------------------------------------
    
    vector<Rect> detections;
    cv::Mat image;
    
    
    for (int i=0; i<300; i++) 
    { 	  
      //get frame
      Camera.grab();
      Camera.retrieve ( image );
      
      //transform image and detect face
      detections.clear();
      resize(image,image,Size(BB_VIDEO_WIDTH,BB_VIDEO_HEIGHT));
      detector.detectMultiScale(image, detections, 1.05, 3, 0, Size(BB_MIN_FACE,BB_MIN_FACE), Size(BB_MAX_FACE,BB_MAX_FACE));
      
      //draw face
      for (int i=0; i<detections.size(); i++)
	rectangle(image, detections[i], cv::Scalar(255,0,0));
      imshow("frame",image);	
      
      if(waitKey(1)!=-1)
      {
	break;
      }
    }
    
    delay(1000);    
    
    
    frown(1);
    frown(-1);
    
    driveHead(90, 60);
    delay(500);
    driveHead(60, 120);
    delay(500);
    driveHead(120, 90);
    delay(500);
  
    //--------------------------------------------RESET--------------------------------------------------
    resetRobot();
    Camera.release();
    
    return 0;
}