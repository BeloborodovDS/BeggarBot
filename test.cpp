#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <opencv2/objdetect.hpp>
#include <raspicam/raspicam_cv.h>
#include <iostream>

using namespace cv;
using namespace std;


int main( int argc, char** argv )
{    
    //Init camera
    raspicam::RaspiCam_Cv Camera;
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

    
    
    //Scale ratio for input vodeo frames
    double shrink = 0.25;

    
    cv::Mat image(960, 1280, CV_8UC3);
    cv::Mat gray(960, 1280, CV_8UC1);
    cv::Mat small(240, 360, CV_8UC1);
    
    for (;;) 
    { 
      try{	  
	  //get frame
	  Camera.grab();
	  Camera.retrieve ( image );
	  
	  //transform image and detect face
	  detections.clear();
	  cvtColor(image, gray, CV_BGR2GRAY);
	  resize(gray, small, Size(360,240));
	  flip(small,small,0);
	  detector.detectMultiScale(small, detections, 1.1, 2, 0, Size(15,15), Size(150,150));
	  
	  //draw face
	  for (int i=0; i<detections.size(); i++)
	    rectangle(small, detections[i], cv::Scalar(255,0,0));
	  imshow("frame",small);
	  
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
    
    return 0;
 
}