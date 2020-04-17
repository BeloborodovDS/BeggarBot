#include "ros/ros.h"
#include "ros/package.h"
#include "beggar_bot/DetectionBox.h"
#include "beggar_bot/DetectionList.h"

#include <opencv2/core/core.hpp>
#include <raspicam/raspicam.h>

#include "ncs_wrapper/vino_wrapper.hpp"
#include "../submodules/sort-cpp/sort-c++/SORTtracker.h"

#include "../include/constants.h"

using namespace cv;
using namespace beggar_bot;

/* function to parse SSD detector output
 * @param predictions: output buffer of SSD net 
 * @param numPred: maximum number of SSD predictions (from net config)
 * @param w,h: target image height and width
 * @param thresh: detection threshold
 * @param probs, boxes: resulting confidences and bounding boxes
 */
void get_detection_boxes(const float* predictions, int numPred, int w, int h, float thresh,
                         vector<float>& probs, vector<Rect_<float>>& boxes)
{
    float score = 0, cls = 0, id = 0;
    
    //predictions holds numPred*7 values
    //data format: image_id, detection_class, detection_confidence, box_normed_x, box_normed_y, box_normed_w, box_normed_h
    for (int i = 0; i < numPred; i++)
    {
        score = predictions[i*7+2];
        cls = predictions[i*7+1];
        id = predictions[i*7];
        if (id>=0 && score>thresh && cls<=1)
        {
            probs.push_back(score);
            boxes.push_back(Rect_<float>(
                predictions[i*7+3] * w, 
                predictions[i*7+4] * h,
                (predictions[i*7+5] - predictions[i*7+3]) * w, 
                (predictions[i*7+6] - predictions[i*7+4]) * h
            ));
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<DetectionList>("camera_state", 1);
    ros::Rate loop_rate(1000);
    
    DetectionList msg;
    DetectionBox box;
  
    //Start communication with NCS
    NCSWrapper NCS(false);
    std::string model_path = ros::package::getPath("beggar_bot") + BB_FACE_MODEL;
    if (!NCS.load_file(model_path))
    {
        ROS_ERROR_STREAM("Camera node: cannot load graph file " + model_path);
        return 0;
    }
    ROS_INFO_STREAM("Camera node: loaded graph file " + model_path);
    
    //Init camera
    //setup camera
    raspicam::RaspiCam Camera;
    Camera.setFormat(raspicam::RASPICAM_FORMAT_BGR);
    Camera.setVideoStabilization(true);
    Camera.setExposure(raspicam::RASPICAM_EXPOSURE_ANTISHAKE);
    Camera.setAWB(raspicam::RASPICAM_AWB_SHADE);
    if ( !Camera.open() ) 
    {
        ROS_ERROR("Camera node: Error opening camera");
        return 0;
    }
    ROS_INFO_STREAM("Camera node: Connected to camera = " + Camera.getId());
    
    SORTtracker tracker(TRACKING_MAX_AGE, TRACKING_MIN_HITS, TRACKING_MIN_IOU);
    bool first_detections = true;  //flag used to init tracker

    int H = NCS.netInputHeight, W = NCS.netInputWidth;  // net input image size
    Mat data_mat(H, W, CV_8UC3);
    
    unsigned char* frame_data = NULL; //raw frame from camera
    float* ncs_output = NULL; //ncs output buffer is provided by wrapper
    
    vector<Rect_<float>> face_vector;
    vector<float> prob_vector;
    vector<TrackingBox> tracked_faces;
    
    int count = 0;
    int64 start = getTickCount();
    double time = 0;
  
    ROS_INFO("Camera node initialized");

    while (ros::ok())
    {
        //get raw frame and create Mat for it
        Camera.grab();           
        frame_data = Camera.getImageBufferData();
        Mat frame(BB_RAW_HEIGHT, BB_RAW_WIDTH, CV_8UC3, frame_data);
        
        //resize frame
        resize(frame, data_mat, Size(W, H), 0, 0, INTER_NEAREST);
        
        //load image into NCS
        if (!NCS.load_tensor_nowait(data_mat))
        {
            ROS_ERROR("Camera node: an error occured in NCS (load tensor)");
            break;
        }
        
        // get result from NCS (blocking)
        if(!NCS.get_result(ncs_output))
        {
            ROS_ERROR("Camera node: an error occured in NCS (get_result)");
            break;
        }
        
        // decode detections from NCS
        face_vector.clear();
        prob_vector.clear();
        get_detection_boxes(ncs_output, NCS.maxNumDetectedFaces, 
                            W, H, 0.2, prob_vector, face_vector);
        
        // if first detections ever: init tracker if possible
        if (face_vector.size()>0 && first_detections)
        {
            tracker.init(face_vector);
            first_detections = false;
        }
        
        // if tracker initialized, track faces
        if (!first_detections)
        {
            tracked_faces.clear();
            tracker.step(face_vector, tracked_faces);
        }
        
        // report FPS
        if (count > 0 and count % 100 == 0) 
        {
            time = (getTickCount() - start) / getTickFrequency();
            ROS_INFO("Camera node: FPS   %f", count / time);
        }
        
        // build ROS message
        msg.count = count;
        msg.detections.clear();
        for (int i = 0; i < tracked_faces.size(); i++) 
        {
            box.x = tracked_faces[i].box.x;
            box.y = tracked_faces[i].box.y;
            box.width = tracked_faces[i].box.width;
            box.height = tracked_faces[i].box.height;
            msg.detections.push_back(box);
        }
    
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        count ++;
    }
    
    Camera.release();

    return 0;
}
