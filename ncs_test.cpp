#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

#include <raspicam/raspicam_cv.h>

#include <iostream>
#include <fstream>

#include <mvnc.h>
#include <ncs_wrapper/ncs_wrapper.hpp>

using namespace cv;
using namespace std;

#define BB_VIDEO_WIDTH                   300
#define BB_VIDEO_HEIGHT                 300
#define NETWORK_OUTPUT_SIZE     707

void get_detection_boxes(float* predictions, int w, int h, float thresh, 
                                                std::vector<float>& probs, std::vector<cv::Rect>& boxes)
{
    int num = predictions[0];
    float score = 0;
    float cls = 0;
    for (int i=1; i<num+1; i++)
    {
      score = predictions[i*7+2];
      cls = predictions[i*7+1];
      if (score>thresh && cls<=1)
      {
            probs.push_back(score);
            boxes.push_back(Rect(predictions[i*7+3]*w, predictions[i*7+4]*h,
                        (predictions[i*7+5]-predictions[i*7+3])*w, 
                        (predictions[i*7+6]-predictions[i*7+4])*h));
      }
    }
}

int main()
{
    //NCS interface
    NCSWrapper NCS(BB_VIDEO_WIDTH*BB_VIDEO_HEIGHT*3, NETWORK_OUTPUT_SIZE);
    
    //Start communication with NCS
    if (!NCS.load_file("./data/ssd-face.graph"))
        return 0;
  
    //Init camera from raspicam
    raspicam::RaspiCam_Cv Camera;
    Camera.set ( CV_CAP_PROP_FORMAT, CV_8UC3); //color
    if ( !Camera.open() ) 
    {
        cout<<"Error opening camera"<<endl;
        return 0;
    }
    cout<<"Connected to camera ="<<Camera.getId() <<endl;
    
    Mat frame;
    Mat resized(BB_VIDEO_HEIGHT, BB_VIDEO_WIDTH, CV_8UC3);
    Mat resized16f(BB_VIDEO_HEIGHT, BB_VIDEO_WIDTH, CV_32FC3);
    resized16f = Scalar(0);
    Camera.grab();
    Camera.retrieve ( frame ); //to get size
    float* result;
    
    //Capture-Render cycle
    int nframes=0;
    int64 start = getTickCount();
    
    vector<Rect> rects;
    vector<float> probs;
    for(;;)
    {
        nframes++;
            
        //load data to NCS
        if(!NCS.load_tensor_nowait((float*)resized16f.data))
        {
            if (NCS.ncsCode == NC_MYRIAD_ERROR)
            {
                char* err = new char [NC_DEBUG_BUFFER_SIZE];
                unsigned int len;
                ncGraphGetOption(NCS.ncsGraph, NC_RO_GRAPH_DEBUG_INFO, err, &len);
                cout<<string(err, len)<<endl;
                delete [] err;
            }
            break;
        }
        
        //draw boxes and render frame
        for (int i=0; i<rects.size(); i++)
        {
            if (probs[i]>0) 
            rectangle(resized, rects[i], Scalar(0,0,255));
        }
        imshow("render", resized);
        
        //Get frame
        Camera.grab();
        Camera.retrieve (frame);
        
        //transform next frame while NCS works
        if (frame.channels()==4)
            cvtColor(frame, frame, CV_BGRA2BGR);
        flip(frame, frame, 1);
        resize(frame, resized, Size(BB_VIDEO_WIDTH, BB_VIDEO_HEIGHT));
        resized.convertTo(resized16f, CV_32F);
        Scalar mean, stddev;
        meanStdDev(resized16f, mean, stddev);
        add(resized16f, -mean, resized16f);
        divide(resized16f, 3.0*stddev, resized16f);
        //resized.convertTo(resized16f, CV_32F, 1/127.5, -1);
        
        //get result from NCS
        if(!NCS.get_result(result))
        {
            if (NCS.ncsCode == NC_MYRIAD_ERROR)
            {
                char* err = new char [NC_DEBUG_BUFFER_SIZE];
                unsigned int len;
                ncGraphGetOption(NCS.ncsGraph, NC_RO_GRAPH_DEBUG_INFO, err, &len);
                cout<<string(err, len)<<endl;
                delete [] err;
            }
            break;
        }
        
        //get boxes and probs
        probs.clear();
        rects.clear();
        get_detection_boxes(result, resized.cols, resized.rows, 0.3, probs, rects);
        
        //Exit if any key pressed
        if (waitKey(1)!=-1)
        {
            break;
        }
    }
    
    //calculate fps
    double time = (getTickCount()-start)/getTickFrequency();
    cout<<"Frame rate: "<<nframes/time<<endl;
}

