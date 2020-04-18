#include <iostream>
#include <fstream>
#include <set>
#include <cstring>
#include <future>
#include <atomic>
#include <unistd.h>

#include "constants.h"

using namespace std;

std::atomic_bool is_running;  //is used to stop threads from main()

float g_headPos; //head position

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

int main( int argc, char** argv )
{    
    //------------------------------------------------INIT-----------------------------------------------------------------
    is_running = true; //will be used to terminate threads
    
    int found_face = BB_NO_FACE;
    
    std::atomic_bool face_processed (true);
    
    //-------------------------------------------------MAIN BODY-----------------------------------------------------------

    
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
    
    return 0;
}
