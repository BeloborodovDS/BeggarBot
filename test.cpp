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
    
    //Convenience vars
    cv::Mat1b gray;
    cv::Mat small;
    
    vector<Rect> detections;

    cv::Mat image;
    
    //Scale ratio for input vodeo frames
    double shrink = 0.4;

    for (;;) 
    { 
	try{
	  //get frame
	  Camera.grab();
	  Camera.retrieve ( image );
	  
	  //transform image and detect face
	  detections.clear();
	  cvtColor(image, gray, CV_BGR2GRAY);
	  resize(gray, small, Size(gray.size[1]*shrink, gray.size[0]*shrink));
	  flip(small,small,0);
	  detector.detectMultiScale(small, detections, 1.15, 3, 0, Size(30,30), Size(200,200));
	  
	  //draw face
	  rectangle(small, detections[0], cv::Scalar(255,0,0));
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
    
    cout<<"resolution: "<<image.rows<<" x "<<image.cols<<endl;
    cout<<"shrink resolution: "<<small.rows<<" x "<<small.cols<<endl;
    
    return 0;
 
}