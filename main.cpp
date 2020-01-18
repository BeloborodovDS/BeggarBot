#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

#include <raspicam/raspicam.h>

#include <iostream>
#include <fstream>
#include <set>
#include <cstring>
#include <future>
#include <atomic>
#include <unistd.h>

#include <pthread.h>

#include "pca9685.h"
#include <wiringPi.h>
#include "submodules/mcp3008/mcp3008Spi.h"

#include <inference_engine.hpp>
#include "ncs_wrapper/vino_wrapper.hpp"
#include "submodules/sort-cpp/sort-c++/SORTtracker.h"

#include "constants.h"

using namespace cv;
using namespace std;
using namespace InferenceEngine;

std::atomic_bool is_running;  //is used to stop threads from main()
pthread_mutex_t face_vector_mutex;    //mutex for shared face vectors (probs and faces)

float g_headPos; //head position

/* Struct to pass to threads
 * camera: pointer to RaspiCam camera
 * buffer: pointer to shared image buffer
 * face_processed: flag to process data only once
 * ncs: pointer to NCSWrapper instance
 * faces, probs: pointers to shared face and prob vectors
 * trfaces: pointers to tracked faces and auxilary values
 */
struct thread_pointers_t
{
    raspicam::RaspiCam *camera;
    unsigned char *buffer;
    std::atomic_bool *face_processed;
    NCSWrapper *ncs;
    vector<Rect> *faces;
    vector<float> *probs;
    vector<TrackingBox> *trfaces;
};

//struct used in a separate thread to read from IR sensors
struct thread_sensors_t
{
    mcp3008Spi *adc;
    std::atomic<float> *values;
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

//drive servo by pulse width value
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
  driveDegs(65, BB_PIN_ARM);
  delay(500);
  driveDegs(90, BB_PIN_ARM);
  delay(500);
  driveDegs(115, BB_PIN_ARM);
  delay(500);
  driveDegs(90, BB_PIN_ARM);
  delay(500);
}


//Drive head from g_headPos to g_headPos+delta_angle with specified speed
//speed is ratio from SERVO_MAX_SPEED, in [0,1]
void driveHead(float delta_angle, float speed = 0.1)
{
  // limits
  if (speed > 1) speed = 1;
  if (speed < 0) speed = 0;
  float an_from = g_headPos, an_to = g_headPos+delta_angle;
  if (an_to > BB_HEAD_MAX_LIMIT) an_to = BB_HEAD_MAX_LIMIT;
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
  g_headPos = an_to;
  
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

//set speed for left and right wheels
void setSpeed(float speed_left, float speed_right)
{
  setSpeedLeft(speed_left);
  setSpeedRight(speed_right);  
}

//rotate platform: > 0: clockwise, < 0: counter-clockwise
void rotatePlatform(float degrees)
{
    float sign = 2*float(degrees >= 0)  - 1;
    setSpeedLeft(sign);
    setSpeedRight(-sign);
    delay(1000 * sign * degrees / BB_DEG_PER_SECOND);
    setSpeedLeft(0);
    setSpeedRight(0);
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

/* function to parse SSD detector output
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
        
    float* ncs_output = NULL; //ncs output buffer is provided by wrapper
    bool success = false;
    
    SORTtracker tracker(TRACKING_MAX_AGE, TRACKING_MIN_HITS, TRACKING_MIN_IOU);
    bool first_detections = true;  //flag used to init tracker
    vector<Rect_<float> > tmp_det;  //convenience vector for type conversion
    
    while(is_running)
    {
        //get raw frame and create Mat for it
        pnt->camera->grab();           
        frame_data = pnt->camera->getImageBufferData();
        Mat frame(BB_RAW_HEIGHT, BB_RAW_WIDTH, CV_8UC3, frame_data);
        
        //Create Mats for existing shared image buffers (for convenience)
        Mat data_mat(H, W, CV_8UC3, pnt->buffer);
        
        //resize frame into shared buffer
        resize(frame, data_mat, Size(W,H), 0, 0, INTER_NEAREST);
        
        //load image into NCS
        success = pnt->ncs->load_tensor_nowait(data_mat);
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
        
        *(pnt->face_processed) = false;
        
        pthread_mutex_unlock(&face_vector_mutex);//________________UNLOCK________________________________
    
        nframes++;
    }
    //report fps and exit
    double time = (getTickCount()-start)/getTickFrequency();
    cout<<"Frame processing FPS: "<<nframes/time<<endl;
    pthread_exit((void*) 0);
}

/* Read values from IR sensors in a thread 
 process them with EWMA*/
void* get_sensors(void* pointers)
{
    float left=0, right=0;
    thread_sensors_t* pnt = (thread_sensors_t*) pointers;
    
    // initial read
    left = analogRead(*(pnt->adc), BB_IR_LEFT) / BB_IR_SCALER_LEFT;
    right = analogRead(*(pnt->adc), BB_IR_RIGHT) / BB_IR_SCALER_RIGHT;
    pnt->values[0] = left;
    pnt->values[1] = right;
    delay(10);
    
    // other reads
    while(is_running)
    {
        // read and update by EWMA
        left = analogRead(*(pnt->adc), BB_IR_LEFT) / BB_IR_SCALER_LEFT;
        right = analogRead(*(pnt->adc), BB_IR_RIGHT) / BB_IR_SCALER_RIGHT;
        left = left*BB_EWMA_GAMMA + (pnt->values[0])*(1-BB_EWMA_GAMMA);
        right = right*BB_EWMA_GAMMA + (pnt->values[1])*(1-BB_EWMA_GAMMA);
        pnt->values[0] = left;
        pnt->values[1] = right;
        delay(10);
    }
}

//set initial position
void resetRobot()
{
  driveDegs(BB_HEAD_INIT_POS, BB_PIN_HEAD);
  g_headPos = BB_HEAD_INIT_POS;
  frown(0);
  driveDegs(45, BB_PIN_ARM);
  delay(2000);
}

// find closest to center face and try to align to it
int follow_face(thread_pointers_t* pointers)
{
  int H = pointers->ncs->netInputHeight, W = pointers->ncs->netInputWidth;  //net input image size
  float y = 0, x = 0, dist=0, mindist=100000, sign = 0;
  TrackingBox trbox, best_trbox;
  
  // wait for another processed frame
  while ( *(pointers->face_processed))
      delay(10);
  *(pointers->face_processed) = true;
  
  // find face closest to frame center
  pthread_mutex_lock(&face_vector_mutex);//________________LOCK_____________________________
  for (int i=0; i<pointers->trfaces->size(); i++)
  {
    trbox = (*(pointers->trfaces))[i];
    y = trbox.box.y + trbox.box.height/2;
    x = trbox.box.x + trbox.box.width/2;
    dist = abs(y - H/2.0) + abs(x - W/2.0);
    if (dist < mindist)
    {
        mindist = dist; 
        best_trbox = trbox;
    }
  }
  pthread_mutex_unlock(&face_vector_mutex);//________________UNLOCK________________________________
  
  // no face detected
  if (mindist >= 100000)
      return BB_NO_FACE;
  
  // calculate displace of face center from frame center
  y = best_trbox.box.y + best_trbox.box.height/2 - H/2.0;
  y = y*BB_VFOV/H;
  sign = float(y>=0)*2 - 1;
  
  x = best_trbox.box.x + best_trbox.box.width/2 - W/2.0;
  x = x*BB_HFOV/W;
  
  // cannot move head further
  if ((g_headPos+sign < BB_HEAD_MIN_LIMIT) or (g_headPos+sign > BB_HEAD_MAX_LIMIT))
      return BB_FACE_LIMIT;
  
  // close enough to center
  if ((abs(y) < BB_FOLLOW_TOLERANCE * H) and (abs(x) < BB_FOLLOW_TOLERANCE * W))
  {
    // too small face
    if (best_trbox.box.height * best_trbox.box.width / (H*W) < BB_FACE_AREA)
      return BB_FACE_FAR;
    else
      return BB_FOUND_FACE;
  }
  
  // rotate platform and move head asynchronously
  auto waitRotation = std::async(std::launch::async, rotatePlatform, x);
  driveHead(y, 0.1);
  waitRotation.wait();
  delay(200);
  
  // wait for next processed frame just in case
  while ( *(pointers->face_processed))
      delay(10);
  *(pointers->face_processed) = true;
  
  // moving robot
  return BB_FACE_BUSY;
}

// set "is_running" as a read from switch
void* track_button(void*)
{
    pinMode(BB_PIN_SWITCH, INPUT);
    pullUpDnControl(BB_PIN_SWITCH, PUD_UP);
    
    int button;
    
    while (is_running)
    {
        button = digitalRead(BB_PIN_SWITCH);
        if (button == 1) is_running = false;
        delay(50);
    }
}

int main( int argc, char** argv )
{    
    //------------------------------------------------INIT-----------------------------------------------------------------
    is_running = true; //will be used to terminate threads
    
    int found_face = BB_NO_FACE;
    
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
    Camera.setFormat(raspicam::RASPICAM_FORMAT_BGR);
    Camera.setVideoStabilization(true);
    Camera.setExposure(raspicam::RASPICAM_EXPOSURE_ANTISHAKE);
    Camera.setAWB(raspicam::RASPICAM_AWB_SHADE);
    
    if ( !Camera.open() ) 
    {
        cout<<"Error opening camera"<<endl;
        return 0;
    }
    cout<<"Connected to camera ="<<Camera.getId() <<endl;
    
    //allocate and reset SHARED image buffers
    //use buffer 0 to write transformed data
    //use buffer 1 to read data
    unsigned char* image_buffer = new unsigned char [HW3]; 
    for (int i=0; i<HW3; i++)  image_buffer[i] = 0;

    //Convenience Mat
    Mat display_frame(H, W, CV_8UC3); // SHARED
    
    //SHARED face and prob vectors
    vector<Rect> face_vector;
    vector<float> prob_vector;
    //SHARED tracked_faces vector
    vector<TrackingBox> tracked_faces;
    
    std::atomic_bool face_processed (true);

    //init all mutex
    pthread_mutex_init(&face_vector_mutex, NULL);
    
    //setup structure to pass to face thread
    thread_pointers_t thread_pointers;
    thread_pointers.buffer = image_buffer;
    thread_pointers.face_processed = &face_processed;
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
    
    std::atomic<float> IR_values[2];
    
    //setup structure to pass to face thread
    thread_sensors_t thread_sensors_pnt;
    thread_sensors_pnt.adc = &ADC;
    thread_sensors_pnt.values = IR_values;
    
    resetRobot();
    cout<<"Robot reset"<<endl;
    
    g_headPos = BB_HEAD_INIT_POS;
    
    //-------------------------------------------------MAIN BODY-----------------------------------------------------------
    
    //launch threads
    pthread_t thread_get_frames, thread_get_sensors, thread_track_button;
    int err = 0;
    err = pthread_create(&thread_get_frames, &threadAttr, get_frames, (void*)&thread_pointers);
    if (err)
        cout<<"Failed to create get_frames process with code "<<err<<endl;
    err = pthread_create(&thread_get_sensors, &threadAttr, get_sensors, (void*)&thread_sensors_pnt);
    if (err)
        cout<<"Failed to create get_sensors process with code "<<err<<endl;
    err = pthread_create(&thread_track_button, &threadAttr, track_button, (void*)NULL);
    if (err)
        cout<<"Failed to create track_button process with code "<<err<<endl;
    
    //cleanup
    pthread_attr_destroy(&threadAttr);
    
    // for navigation
    float left=0, right=0, alpha=0, sign=1;
    int roam_counter = 0;
    
    //main cycle
    while(is_running)
    {
        // while face is seen, align to it
        found_face = follow_face(&thread_pointers);
        while (found_face == BB_FACE_BUSY and is_running)
            found_face = follow_face(&thread_pointers);
        
        // read sensors
        left = IR_values[0];
        right = IR_values[1];
        
        // face detected => perform
        if (found_face == BB_FOUND_FACE)
        {
            // stop, shake and detect face again
            roam_counter = 0;
            setSpeed(0, 0);
            shake();
            delay(1000);
            found_face = follow_face(&thread_pointers);
            // if face is gone, angry frown, else shake again
            if (found_face == BB_NO_FACE)
                frown(-1);
            else
            {
                frown(1);
                shake();
                delay(1000);
            }
            // turn and go
            driveHead(BB_HEAD_INIT_POS - g_headPos);
            rotatePlatform(180);
            frown(0);
        }
        // if face over limit => angry frown
        else if (found_face == BB_FACE_LIMIT)
        {
            roam_counter = 0;
            setSpeed(0, 0);
            frown(-1);
            delay(1000);
            rotatePlatform(180);
        }
        // no face of face too far: random exploration
        else
        {     
            alpha = rand() % 91; // [0, 90]
            sign = (rand() % 2) * 2 - 1; // {-1, 1}
            
            // obstacles far: go forward
            if (left < 1 and right < 1)
            {
                setSpeed(1, 1);
                roam_counter ++;
            }
            // obstacles close: random rotate [90, 180] u [-180, -90]
            else if (left >=1 and right >=1)
            {
                rotatePlatform((90 + alpha) * sign);
                roam_counter = 0;
                driveHead(BB_HEAD_INIT_POS - g_headPos);
            }
            // obstacles to the right: rotate left 
            else if (left > right)
            {
                rotatePlatform(alpha);
                roam_counter = 0;
                driveHead(BB_HEAD_INIT_POS - g_headPos);
            }
            // obstacles to the left: rotate right
            else
            {
                rotatePlatform(-alpha);
                roam_counter = 0;
                driveHead(BB_HEAD_INIT_POS - g_headPos);
            }
            
            // if going forward for too long (stuck): go back a little and random rotate
            if (roam_counter > BB_ROAM_LIMIT)
            {
                roam_counter = 0;
                setSpeed(-1, -1);
                delay(1000);
                rotatePlatform((90 + alpha) * sign);
                setSpeed(1, 1);
            }
            delay(10);
        }
    }
    
    setSpeed(0, 0);
    
    //join threads
    err = pthread_join(thread_get_frames, NULL);
    if (err)
        cout<<"Failed to join get_frames process with code "<<err<<endl;
    err = pthread_join(thread_get_sensors, NULL);
    if (err)
        cout<<"Failed to join get_sensors process with code "<<err<<endl;
    err = pthread_join(thread_track_button, NULL);
    if (err)
        cout<<"Failed to join track_button process with code "<<err<<endl;


    //--------------------------------------------RESET--------------------------------------------------
    delete [] image_buffer;
    pthread_mutex_destroy(&face_vector_mutex);
    
    resetRobot();
    cout<<"Final reset."<<endl;
    Camera.release();
    
    pca9685PWMReset(pca_fd);
    
    return 0;
}
