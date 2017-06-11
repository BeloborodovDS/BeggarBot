#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <opencv2/objdetect.hpp>
#include <raspicam/raspicam_cv.h>
#include <iostream>

using namespace cv;
using namespace std;

#define BB_VIDEO_WIDTH 		360
#define BB_VIDEO_HEIGHT 	240
#define BB_MIN_FACE		15
#define BB_MAX_FACE		150

#define BB_DO_FLIP		1

int main( int argc, char** argv )
{    
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
    
    return 0;
 
}