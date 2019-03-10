#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

#include <raspicam/raspicam.h>

#include <iostream>
#include <fstream>
#include <set>
#include <cstring>
#include <unistd.h>

#include <pthread.h>

#include <inference_engine.hpp>
#include "ncs_wrapper/vino_wrapper.hpp"

#include "submodules/sort-cpp/sort-c++/SORTtracker.h"

using namespace cv;
using namespace std;
using namespace InferenceEngine;

#define BB_RAW_WIDTH                     1280
#define BB_RAW_HEIGHT                    960
#define BB_FACE_MODEL                   "./data/face_vino"

#define TRACKING_MAX_AGE          8
#define TRACKING_MIN_HITS          5
#define TRACKING_MIN_IOU            0.05
#define TRACKING_NUM_COLORS  20


volatile bool is_running;  //is used to stop threads from main()
pthread_mutex_t image_buffer_mutex; //mutex for shared image buffer
pthread_mutex_t face_vector_mutex;    //mutex for shared face vectors (probs and faces)

/* Struct to pass to threads
 * camera: pointer to RaspiCam camera
 * buffers: pointer to shared image buffers
 * bufcnt: counter of filled buffers
 * ncs: pointer to NCSWrapper instance
 * faces, probs: pointers to shared face and prob vectors
 * trfaces, trprobs: pointers to tracked faces and auxilary values
 */
struct thread_pointers_t
{
    raspicam::RaspiCam *camera;
    unsigned char **buffers;
    int *bufcnt;
    NCSWrapper *ncs;
    vector<Rect> *faces;
    vector<float> *probs;
    vector<TrackingBox> *trfaces;
};

//calc IOU between two rects
float rect_iou(Rect rect1, Rect rect2)
{
    float inter = (rect1 & rect2).area();
    if (inter <= 0) return 0;
    return inter/(rect1.area() + rect2.area() - inter);
}

//calc distance between two rects
float rect_dist(Rect rect1, Rect rect2)
{
    Point p1 = 0.5*(rect1.tl()+rect1.br());
    Point p2 = 0.5*(rect2.tl()+rect2.br());
    return sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
}

/* function to parse SSD fetector output
 * @param predictions: output buffer of SSD net 
 * @param numPred: maximum number of SSD predictions (from net config)
 * @param w,h: target image height and width
 * @param thresh: detection threshold
 * @param probs, boxes: resulting confidences and bounding boxes
 */
void get_detection_boxes(const float* predictions, int numPred, int w, int h, float thresh, 
                                                std::vector<float>& probs, std::vector<cv::Rect>& boxes)
{
    float score = 0;
    float cls = 0;
    float id = 0;
    
    //predictions holds numPred*7 values
    //data format: image_id, detection_class, detection_confidence, box_normed_x, box_normed_y, box_normed_w, box_normed_h
    for (int i=0; i<numPred; i++)
    {
      score = predictions[i*7+2];
      cls = predictions[i*7+1];
      id = predictions[i*7  ];
      if (id>=0 && score>thresh && cls<=1)
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
    int H = pnt->ncs->netInputHeight, W = pnt->ncs->netInputWidth;  //net input image size
    
    unsigned char* frame_data = NULL; //raw frame
    unsigned char *swap = NULL;  //for swapping buffers
    
    while(is_running)
    {
        if (*(pnt->bufcnt)<1)
        {
            //get raw frame and create Mat for it
            pnt->camera->grab();           
            frame_data = pnt->camera->getImageBufferData();
            Mat frame(BB_RAW_HEIGHT, BB_RAW_WIDTH, CV_8UC3, frame_data);
            
            //Create Mats for existing shared image buffers (for convenience)
            Mat resized_frame(H, W, CV_8UC3, pnt->buffers[0]);
            
            //resize frame into shared buffer
            resize(frame, resized_frame, Size(W,H), 0, 0, INTER_NEAREST);
            
            //swap buffers
            //pthread_mutex_lock(&image_buffer_mutex);
            swap = pnt->buffers[0];
            pnt->buffers[0] = pnt->buffers[1];
            pnt->buffers[1] = swap;
            *(pnt->bufcnt)+=1;
            //pthread_mutex_unlock(&image_buffer_mutex);
            
            nframes++;
        }
        //usleep(1000);  //we dont need it since grab() blocks and waits
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
    int H = pnt->ncs->netInputHeight, W = pnt->ncs->netInputWidth;  //net input image size
    float* ncs_output = NULL; //ncs output buffer is provided by wrapper
    bool success = false;
    
    SORTtracker tracker(TRACKING_MAX_AGE, TRACKING_MIN_HITS, TRACKING_MIN_IOU);
    bool first_detections = true;  //flag used to init tracker
    vector<Rect_<float> > tmp_det;  //convenience vector for type conversion
    
    while(is_running)
    {
        if (*(pnt->bufcnt)>=1)
        {            
            //load image into NCS
            //pthread_mutex_lock(&image_buffer_mutex);
            Mat data_mat(H, W, CV_8UC3, pnt->buffers[1]);
            success = pnt->ncs->load_tensor_nowait(data_mat);
            *(pnt->bufcnt)-=1;
            //pthread_mutex_unlock(&image_buffer_mutex);
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
            pthread_mutex_lock(&face_vector_mutex);//________________LOCK_____________________________
            
            //get detections from NCS
            pnt->faces->clear();
            pnt->probs->clear();
            tmp_det.clear();
            get_detection_boxes(ncs_output, pnt->ncs->maxNumDetectedFaces, 
                                                    W, H, 0.2, *(pnt->probs), *(pnt->faces));
            
            //copy to tmp_det to transform type
            for (int i=0; i<pnt->faces->size(); i++)
            {
                Rect_<float> r = (*(pnt->faces))[i];
                if ((*(pnt->probs))[i] > 0)
                    tmp_det.push_back(r);
            }
            
            //if first detections ever: init tracker if possible
            if (pnt->faces->size()>0 && first_detections)
            {
                tracker.init(tmp_det);
                first_detections = false;
            }
            
            //if tracker initialized, track faces
            if (!first_detections)
            {
                pnt->trfaces->clear();
                tracker.step(tmp_det, *(pnt->trfaces));
            }
            
            pthread_mutex_unlock(&face_vector_mutex);//________________UNLOCK________________________________

            nframes++;
        }
        //usleep(1000);  //we dont need it since get_result() blocks and waits
    }
    //report fps and exit
    double time = (getTickCount()-start)/getTickFrequency();
    cout<<"Face detection FPS: "<<nframes/time<<endl;
    pthread_exit((void*) 0);
}

int main()
{
    is_running = true; //will be used to terminate threads
    
    //----------------------------------NCS-WRAPPER----------------------
    //NCS interface
    NCSWrapper NCS(true);
    //Start communication with NCS
    if (!NCS.load_file(BB_FACE_MODEL))
    {
        cout<<"Cannot load graph file"<<endl;
        return 0;
    }
    
    //get image size from neural network
    int H=NCS.netInputHeight, W=NCS.netInputWidth;
    int HW3 =H*W*3;
    
    //-----------------------------------CAMERA---------------------------------------
    //setup camera
    raspicam::RaspiCam Camera;
    Camera.setContrast(100);//50
    Camera.setISO(800);//500
    Camera.setSaturation(-100);//-20
    Camera.setVideoStabilization(true);
    Camera.setExposure(raspicam::RASPICAM_EXPOSURE_ANTISHAKE);
    Camera.setAWB(raspicam::RASPICAM_AWB_AUTO);
    
    if(!Camera.open())
    {
        cout<<"Failed to open camera\n";
        return 0;
    }
    cout<<"Connected to camera "<<Camera.getId()<<endl;

    //-----------------------------------------------------------BUFFERS--------------------------------------
    //allocate and reset SHARED image buffers
    //use buffer 0 to write transformed data
    //use buffer 1 to read data
    unsigned char* image_buffers[2]; 
    int buffer_count = 0;
    for (int k=0; k<2; k++)
    {
        image_buffers[k] = new unsigned char [HW3];
        for (int i=0; i<HW3; i++)  image_buffers[k][i] = 0;
    }
    
    //Convenience Mat
    Mat display_frame(H, W, CV_8UC3); // SHARED
    
    //SHARED face and prob vectors
    vector<Rect> face_vector;
    vector<float> prob_vector;
    //SHARED tracked_faces vector
    vector<TrackingBox> tracked_faces;
    
    //----------------------------------------------THREADS-------------------------------------
    //init all mutex
    pthread_mutex_init(&image_buffer_mutex, NULL);
    pthread_mutex_init(&face_vector_mutex, NULL);
    
    //setup structure to pass to threads
    thread_pointers_t thread_pointers;
    thread_pointers.buffers = image_buffers;
    thread_pointers.bufcnt = &buffer_count;
    thread_pointers.camera = &Camera;
    thread_pointers.ncs = &NCS;
    thread_pointers.faces = &face_vector;
    thread_pointers.probs = &prob_vector;
    thread_pointers.trfaces = &tracked_faces;
    
    //set threads to be JOINABLE
    pthread_attr_t threadAttr;
    pthread_attr_init(&threadAttr);
    pthread_attr_setdetachstate(&threadAttr, PTHREAD_CREATE_JOINABLE);
    
    //------------------------------------------------MAIN---------------------------------
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
    
    //colors for faces ids
    RNG rng(0xFFFFFFFF);
    Scalar_<int> colors[TRACKING_NUM_COLORS];
    for (int i = 0; i < TRACKING_NUM_COLORS; i++)
        rng.fill(colors[i], RNG::UNIFORM, 0, 256);
    
    //rendering cycle
    for(;;)
    {
        memcpy(display_frame.data, image_buffers[1], HW3*sizeof(unsigned char));
        
        //draw faces from shared vectors
        pthread_mutex_lock(&face_vector_mutex);
        
        for (int i=0; i<tracked_faces.size(); i++)
        {
            double alpha = ((float)TRACKING_MAX_AGE - tracked_faces[i].age)/TRACKING_MAX_AGE;
            Scalar_<int> intcol = colors[tracked_faces[i].id % TRACKING_NUM_COLORS];
            Scalar col = Scalar(intcol[0],intcol[1],intcol[2]);
            col = alpha*col + (1-alpha)*Scalar(0,0,0);
            rectangle(display_frame, tracked_faces[i].box, col, 3); 
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
    
    //----------------------------CLEANUP------------------------------------------
    
    Camera.release();
    
    //cleanup
    delete [] image_buffers[0];
    delete [] image_buffers[1];
    
    pthread_mutex_destroy(&image_buffer_mutex);
    pthread_mutex_destroy(&face_vector_mutex);
}

