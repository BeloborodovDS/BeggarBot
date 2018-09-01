#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

#include <raspicam/raspicam.h>

#include <iostream>
#include <fstream>

#include <pthread.h>
#include <unistd.h>

#include <mvnc.h>
#include "ncs_wrapper/ncs_wrapper.hpp"

using namespace cv;
using namespace std;

#define BB_VIDEO_WIDTH                   300
#define BB_VIDEO_HEIGHT                 300
#define BB_RAW_WIDTH                     1280
#define BB_RAW_HEIGHT                    960
#define NETWORK_OUTPUT_SIZE     707

volatile bool is_running;  //used to stop threads from main()
pthread_mutex_t image_buffer_mutex; //mutex for shared image buffer
pthread_mutex_t face_vector_mutex;    //mutex for shared face vectors (probs and faces)

/* Struct to pass to threads
 * camera: pointer to RaspiCam camera
 * image: pointer to shared image buffer
 * ncs: pointer to NCSWrapper instance
 * faces, probs: pointers to shared face and prob vectors
 */
struct thread_pointers_t
{
    raspicam::RaspiCam *camera;
    float *image;
    NCSWrapper *ncs;
    vector<Rect> *faces;
    vector<float> *probs;
};

/*Calculate detected bboxes and corresponding probabilities from NCS output 
 * predictions: raw output from NCS (pointer)
 * w,h: display image width and height
 * thresh: include bbox if only its probability is >= thresh
 * probs: empty vector for returned probabilities
 * boxes: empty vector for returned bboxes
 */
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

/* Threaded frame read and processing for NCS input
 */
void* get_frames(void* pointers)
{
    //fps evaluation
    int nframes=0;
    int64 start = getTickCount();
    
    thread_pointers_t* pnt = (thread_pointers_t*) pointers;
    unsigned char* frame_data; //raw frame
    Mat resized_frame(BB_VIDEO_HEIGHT, BB_VIDEO_WIDTH, CV_8UC3); //resized frame
    
    //Create Mat for existing shared image buffer (for convenience)
    pthread_mutex_lock(&image_buffer_mutex);
    Mat float_frame(BB_VIDEO_HEIGHT, BB_VIDEO_WIDTH, CV_32FC3, pnt->image);
    pthread_mutex_unlock(&image_buffer_mutex);
    
    while(is_running)
    {
        //get raw frame and create Mat for it
        pnt->camera->grab();
        frame_data = pnt->camera->getImageBufferData();
        Mat frame(BB_RAW_HEIGHT, BB_RAW_WIDTH, CV_8UC3, frame_data);
        
        //resize frame
        resize(frame, resized_frame, Size(BB_VIDEO_WIDTH,BB_VIDEO_HEIGHT));
        
        //cast resized frame to [-1.0, 1.0] with shared buffer destination
        pthread_mutex_lock(&image_buffer_mutex);
        resized_frame.convertTo(float_frame, CV_32F, 1/127.5, -1);
        pthread_mutex_unlock(&image_buffer_mutex);
        
        nframes++;
        usleep(1000);
    }
    //report fps and exit
    double time = (getTickCount()-start)/getTickFrequency();
    cout<<"Frame processing FPS: "<<nframes/time<<endl;
    pthread_exit((void*) 0);
}

/* Threaded face detection on NCS
 */
void* detect_faces(void* pointers)
{
    //fps evaluation
    int nframes=0;
    int64 start = getTickCount();
    
    thread_pointers_t* pnt = (thread_pointers_t*) pointers;
    float* ncs_output = NULL; //ncs output buffer is provided by wrapper
    bool success = false;
    
    while(is_running)
    {
        //load image into NCS
        pthread_mutex_lock(&image_buffer_mutex);
        success = pnt->ncs->load_tensor_nowait(pnt->image);
        pthread_mutex_unlock(&image_buffer_mutex);
        if(!success)
        {
            pnt->ncs->print_error_code();
            break;
        }
        //now LOCK until result arrives
        if(!pnt->ncs->get_result(ncs_output))
        {
            pnt->ncs->print_error_code();
            break;
        }
        //get result into shared vactors
        pthread_mutex_lock(&face_vector_mutex);
        pnt->faces->clear();
        pnt->probs->clear();
        get_detection_boxes(ncs_output, BB_VIDEO_WIDTH, BB_VIDEO_HEIGHT, 0.3, *(pnt->probs), *(pnt->faces));
        pthread_mutex_unlock(&face_vector_mutex);
        
        nframes++;
        usleep(1000);
    }
    //report fps and exit
    double time = (getTickCount()-start)/getTickFrequency();
    cout<<"Face detection FPS: "<<nframes/time<<endl;
    pthread_exit((void*) 0);
}

int main()
{
    is_running = true; //will be used to terminate threads
    
    //init all mutex
    pthread_mutex_init(&image_buffer_mutex, NULL);
    pthread_mutex_init(&face_vector_mutex, NULL);
    
    //allocate and reset SHARED image buffer
    float* image_buffer = new float [BB_VIDEO_WIDTH*BB_VIDEO_HEIGHT*3];
    for (int i=0; i<BB_VIDEO_WIDTH*BB_VIDEO_HEIGHT*3; i++)
        image_buffer[i] = 0;
    
    //Convenience Mats
    Mat display_frame(BB_VIDEO_HEIGHT, BB_VIDEO_WIDTH, CV_32FC3);
    Mat float_frame(BB_VIDEO_HEIGHT, BB_VIDEO_WIDTH, CV_32FC3, image_buffer); // SHARED
    
    //SHARED face and prob vectors
    vector<Rect> face_vector;
    vector<float> prob_vector;
    
    //setup camera
    raspicam::RaspiCam Camera;
    Camera.setContrast(50);
    Camera.setISO(500);
    Camera.setSaturation(-20);
    Camera.setVideoStabilization(true);
    Camera.setExposure(raspicam::RASPICAM_EXPOSURE_ANTISHAKE);
    Camera.setAWB(raspicam::RASPICAM_AWB_AUTO);
    
    if(!Camera.open())
    {
        cout<<"Failed to open camera\n";
    }
    else
    {
        cout<<"Connected to camera "<<Camera.getId()<<endl;
        
        //NCS interface
        NCSWrapper NCS(BB_VIDEO_WIDTH*BB_VIDEO_HEIGHT*3, NETWORK_OUTPUT_SIZE);
        //Start communication with NCS
        if (!NCS.load_file("./data/ssd-face.graph"))
        {
            cout<<"Cannot load graph file"<<endl;
        }
        else
        {
            //setup structure to pass to threads
            thread_pointers_t thread_pointers;
            thread_pointers.image = image_buffer;
            thread_pointers.camera = &Camera;
            thread_pointers.ncs = &NCS;
            thread_pointers.faces = &face_vector;
            thread_pointers.probs = &prob_vector;
            
            //set threads to be JOINABLE
            pthread_attr_t threadAttr;
            pthread_attr_init(&threadAttr);
            pthread_attr_setdetachstate(&threadAttr, PTHREAD_CREATE_JOINABLE);
            
            //launch threads
            pthread_t thread_get_frames, thread_detect_faces;
            int err = 0;
            err = pthread_create(&thread_get_frames, &threadAttr, get_frames, (void*)&thread_pointers);
            if (err)
                cout<<"Failed to create get_frames process with code "<<err<<endl;
            err = pthread_create(&thread_detect_faces, &threadAttr, detect_faces, (void*)&thread_pointers);
            if (err)
                cout<<"Failed to create detect_faces process with code "<<err<<endl;
            
            //cleanup
            pthread_attr_destroy(&threadAttr);
            
            //rendering cycle
            for(;;)
            {
                //get shared image and convert it
                pthread_mutex_lock(&image_buffer_mutex);
                float_frame.convertTo(display_frame, CV_32F, 0.5, 0.5);
                pthread_mutex_unlock(&image_buffer_mutex);
                
                //draw faces from shared vectors
                pthread_mutex_lock(&face_vector_mutex);
                for (int i=0; i<face_vector.size(); i++)
                {
                    if (prob_vector[i]>0) 
                        rectangle(display_frame, face_vector[i], Scalar(0,0,1.0));
                }
                pthread_mutex_unlock(&face_vector_mutex);
                
                //display and wait for key
                imshow("frame", display_frame);
                usleep(40000);
                if(waitKey(1)!=-1)
                {
                    break;
                }
            }
            is_running = false;
            
            //join threads
            err = pthread_join(thread_get_frames, NULL);
            if (err)
                cout<<"Failed to join get_frames process with code "<<err<<endl;
            err = pthread_join(thread_detect_faces, NULL);
            if (err)
                cout<<"Failed to join detect_frames process with code "<<err<<endl;
        }
        
        Camera.release();
    }
    
    //cleanup
    delete [] image_buffer;
    
    pthread_mutex_destroy(&image_buffer_mutex);
    pthread_mutex_destroy(&face_vector_mutex);
}

