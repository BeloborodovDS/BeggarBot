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

#include "pca9685.h"
#include <wiringPi.h>
#include "submodules/mcp3008/mcp3008Spi.h"

#include <inference_engine.hpp>
#include "ncs_wrapper/vino_wrapper.hpp"
#include "submodules/sort-cpp/sort-c++/SORTtracker.h"


using namespace cv;
using namespace std;
using namespace InferenceEngine;

//Face detection
#define BB_RAW_WIDTH                        1280
#define BB_RAW_HEIGHT                       960
#define BB_FACE_MODEL                       "./data/face_vino"

//Tracking
#define TRACKING_MAX_AGE          8
#define TRACKING_MIN_HITS          5
#define TRACKING_MIN_IOU            0.05
#define TRACKING_NUM_COLORS  20

//pca9685 defines
#define BB_PCA_PIN_BASE 	300
#define BB_PCA_MAX_PWM 		4096 //max ticks in PWM signal
#define BB_PCA_HERTZ 		50

//calibrated: for SG90
#define BB_SERVO_MS_MIN 	0.6
#define BB_SERVO_MS_MAX 	2.4
#define BB_SERVO_MAX_SPEED	0.6 //degrees per ms - from datasheet

//from datasheet: for continuous rotation FS5103R
#define BB_CONT_MS_MIN		1.0
#define BB_CONT_MS_MAX		2.0

#define BB_HEAD_DT		10 //ms time span 
#define BB_HEAD_MAX_STEP	BB_HEAD_DT * BB_SERVO_MAX_SPEED
#define BB_HEAD_INIT_POS	50.0 //initial angle
#define BB_HEAD_MIN_LIMIT	40.0
#define BB_HEAD_MAX_LIMIT	135.0

//PINS
#define BB_PIN_EYEBROW 		BB_PCA_PIN_BASE  //eyebrow servo at pin location 0 on PCA controller
#define BB_PIN_HEAD		BB_PCA_PIN_BASE + 1 //head servo at pin location 1 on PCA controller
#define BB_PIN_ARM		BB_PCA_PIN_BASE + 2 //--
#define BB_PIN_LEFT_MOTOR	BB_PCA_PIN_BASE + 3
#define BB_PIN_RIGHT_MOTOR	BB_PCA_PIN_BASE + 4
#define BB_PIN_PCA		BB_PCA_PIN_BASE + 16 //all pins on PCA controller 

//ADC
#define BB_IR_LEFT		0	//Infrared sensors
#define BB_IR_RIGHT		1

volatile bool is_running;  //is used to stop threads from main()
pthread_mutex_t image_buffer_mutex; //mutex for shared image buffer
pthread_mutex_t face_vector_mutex;    //mutex for shared face vectors (probs and faces)

float g_headPos; //head position

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

//drive servo PIN at angle ANGLE for SG90 servo
//angle: [0,180], pin: PIN_BASE(controller)+SERVO_NUM(0-15) or PIN_BASE(controller)+16 for all servos
void driveDegs(float angle, int pin)
{
  if (angle<0) angle = 0;
  if (angle>180) angle = 180;
  int ticks = (int)(BB_PCA_MAX_PWM * (angle/180.0f*(BB_SERVO_MS_MAX-BB_SERVO_MS_MIN) + BB_SERVO_MS_MIN) / 20.0f + 0.5); //20ms is pulse width (~50 hz)
  pwmWrite(pin, ticks);
}

void driveMs(float Ms, int pin)
{
  float cycleMs = 1000.0f / BB_PCA_HERTZ;
  int ticks = (int)(BB_PCA_MAX_PWM * Ms / cycleMs + 0.5f);
  pwmWrite(pin, ticks);
}

//move eyebrows: 
// 0 for neutral expression
// >0 for kind expression
// <0 for angry expression
void frown(int mode)
{
  if(mode==0)
    driveDegs(60, BB_PIN_EYEBROW);
  else if (mode>0)
    driveDegs(25, BB_PIN_EYEBROW);
  else
    driveDegs(100, BB_PIN_EYEBROW);
  delay(1000);
}

//shake hand
void shake()
{
  driveDegs(70, BB_PIN_ARM);
  delay(400);
  driveDegs(110, BB_PIN_ARM);
  delay(400);
  driveDegs(70, BB_PIN_ARM);
  delay(400);
  driveDegs(110, BB_PIN_ARM);
  delay(400);
}


//Drive head from angle an_from to angle an_to with specified speed
//speed is ratio from SERVO_MAX_SPEED, in [0,1]
//!!! valid an_from value must be provided
void driveHead(float an_from, float an_to, float speed = 0.1)
{
  if (speed > 1) speed = 1;
  if (speed < 0) speed = 0;
  if (an_to > BB_HEAD_MAX_LIMIT) an_to = BB_HEAD_MAX_LIMIT; //angle is limited in [45, 135] for safety reasons
  if (an_to < BB_HEAD_MIN_LIMIT) an_to = BB_HEAD_MIN_LIMIT;
  if (an_from < 0) an_from = 0;
  if (an_from > 180) an_from = 180;
  
  float step = BB_HEAD_MAX_STEP*speed; //deg step for each time span
  
  if (an_from < an_to)  
    while(an_from < an_to) //forward
    {
      an_from += step;
      driveDegs(an_from, BB_PIN_HEAD);
      delay(BB_HEAD_DT);
    }
  else
    while(an_from > an_to) //backwards
    {
      an_from -= step;
      driveDegs(an_from, BB_PIN_HEAD);
      delay(BB_HEAD_DT);
    }
    
  //fix imprecise position
  driveDegs(an_to, BB_PIN_HEAD);
  delay(BB_HEAD_DT);
}

//speed: [-1, 1]: 1 => full forward, -1 => full backwards 
void setSpeedLeft(float speed)
{
  if (speed > 1) speed = 1;
  if (speed < -1) speed = -1;
  int ticks = (int)(BB_PCA_MAX_PWM * (0.5*(speed+1)*(BB_CONT_MS_MAX-BB_CONT_MS_MIN) + BB_CONT_MS_MIN) / 20.0f + 0.5); //20ms is pulse width (~50 hz)
  pwmWrite(BB_PIN_LEFT_MOTOR, ticks);
}

//speed: [-1, 1]: 1 => full forward, -1 => full backwards 
void setSpeedRight(float speed)
{
  if (speed > 1) speed = 1;
  if (speed < -1) speed = -1;
  speed *= -1; //symmetry
  int ticks = (int)(BB_PCA_MAX_PWM * (0.5*(speed+1)*(BB_CONT_MS_MAX-BB_CONT_MS_MIN) + BB_CONT_MS_MIN) / 20.0f + 0.5); //20ms is pulse width (~50 hz)
  pwmWrite(BB_PIN_RIGHT_MOTOR, ticks);
}

//analog read from MCP3008 ADC chip
//adc: mcp3008Spi object
//channel: 0..7
unsigned int analogRead(mcp3008Spi &adc, unsigned char channel)
{
  unsigned char spi_data[3];
  unsigned int val = 0;
  
  if (channel > 8) return -1;
  //write sequence
  spi_data[0] = 1;  //start bit
  spi_data[1] = 0b10000000 |( channel << 4); //mode and channel
  spi_data[2] = 0; //anything
  adc.spiWriteRead(spi_data, sizeof(spi_data) );
  //read value, combine last two bits of second byte with whole third byte
  val = 0;
  val = (spi_data[1]<< 8) & 0b1100000000; 
  val |=  (spi_data[2] & 0xff);
  
  return val;
}

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


void* simpleHeadMove(void* nothing)
{
  driveHead(BB_HEAD_INIT_POS, 120);
  usleep(500);
  driveHead(120, BB_HEAD_INIT_POS);
  usleep(500);
  pthread_exit(NULL);
}

//set initial position
void resetRobot()
{
  driveDegs(BB_HEAD_INIT_POS, BB_PIN_HEAD);
  frown(0);
  driveDegs(90, BB_PIN_ARM);
  delay(2000);
}



int main( int argc, char** argv )
{    
    //------------------------------------------------INIT-----------------------------------------------------------------
    is_running = true; //will be used to terminate threads
    
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
    
    //Init camera
    //setup camera
    raspicam::RaspiCam Camera;
    Camera.setContrast(100);//50
    Camera.setISO(800);//500
    Camera.setSaturation(-100);//-20
    Camera.setVideoStabilization(true);
    Camera.setExposure(raspicam::RASPICAM_EXPOSURE_ANTISHAKE);
    Camera.setAWB(raspicam::RASPICAM_AWB_AUTO);
    
    if ( !Camera.open() ) 
    {
        cout<<"Error opening camera"<<endl;
        return 0;
    }
    cout<<"Connected to camera ="<<Camera.getId() <<endl;
    
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
  
    //setup WiringPi
    wiringPiSetup();
    
    // Setup PCA with pinbase 300 and i2c location 0x40 (default for pca9685)
    // PWM period for SG90 servos is 20ms (50Hz)
    int pca_fd = pca9685Setup(BB_PCA_PIN_BASE, 0x40, BB_PCA_HERTZ);
    if (pca_fd < 0)
    {
        cout<<"Error in init PCA9685!"<<endl;
        return 0;
    }
    // Reset all output
    pca9685PWMReset(pca_fd);
    cout<<"PCA9685 controller connected"<<endl;
     
    //declare and init MCP3008 analog-digital converter with default params
    mcp3008Spi ADC;
    cout<<"MCP3008 chip init"<<endl;
    
    resetRobot();
    cout<<"Robot reset"<<endl;
    
    g_headPos = BB_HEAD_INIT_POS;
    
    //-------------------------------------------------MAIN BODY-----------------------------------------------------------
    
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
        
    delay(1000);    
    
    frown(1);
    frown(-1);
    
    driveHead(BB_HEAD_INIT_POS, 120);
    delay(500);
    driveHead(120, BB_HEAD_INIT_POS);
    delay(500);

    shake();
    
    setSpeedLeft(1);
    setSpeedRight(1);
    delay(1000);
    setSpeedLeft(0);
    setSpeedRight(0);
    delay(1000);
    setSpeedLeft(-1);
    setSpeedRight(-1);
    delay(1000);
    setSpeedLeft(0);
    setSpeedRight(0);
    
    //test IR sensors
    int val1, val2;
    for (int i=0; i<100; i++)
    {
      val1 = analogRead(ADC, BB_IR_LEFT);
      val2 = analogRead(ADC, BB_IR_RIGHT);
      cout<<val1<<"  "<<val2<<endl;
      delay(100);
    }

    //--------------------------------------------RESET--------------------------------------------------
    delete [] image_buffers[0];
    delete [] image_buffers[1];
    
    pthread_mutex_destroy(&image_buffer_mutex);
    pthread_mutex_destroy(&face_vector_mutex);
    
    resetRobot();
    cout<<"Final reset."<<endl;
    Camera.release();
    
    return 0;
}