
#include "ros/ros.h"
#include "ark_task_1/board_pose.h"
#include "ark_task_1/danger_region.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

// %EndTag(MSG_HEADER)%

#include <iostream>

using namespace std;
using namespace cv;


const int max_value_H = 360/2;
const int max_value = 255;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";
int low_H = 0, low_S = 125, low_V = 15;
int high_H = 41, high_S = 185, high_V = 240;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ark_task_1::danger_region>("danger_region");
  ark_task_1::danger_region srv;
 
  ros::Publisher chatter_pub = n.advertise<ark_task_1::board_pose>("check_pose", 1000);
  ros::Rate loop_rate(30);

  VideoCapture cap("/home/archit/new.avi"); 
    
  if(!cap.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }
  namedWindow(window_capture_name);
  namedWindow(window_detection_name);
  // Trackbars to set thresholds for HSV values
  Mat frame, frame_HSV, croppedImage, frame_threshold;

  int count = 0;
  float x = 0, y= 0, lx=0, ly=0;
  int lf = -1;
  int cf=0;
  while (ros::ok())
  {
    cf++;
    ark_task_1::board_pose msg;

    Mat frame;
    
    cap >> frame;
  
    if (frame.empty())
      break;
      cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
    // Detect the object based on HSV Range Values
    inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
    Rect s = boundingRect(frame_threshold);
    if(s.height == 0 || s.width == 0)
    {
      imshow(window_capture_name, frame);
      x=lx, y=ly;
    }
    else
    {
      erode(frame_threshold,frame_threshold,Mat(),Point(-1,-1),10);
      dilate(frame_threshold,frame_threshold,Mat(),Point(-1,-1),10);
      croppedImage = frame(s);
      vector<Point2f> corners;

      int r =findChessboardCorners(croppedImage, cvSize(5,8), corners, CALIB_CB_FAST_CHECK);
      drawChessboardCorners(croppedImage, cvSize(5,8), Mat(corners), r);
      x=0,y=0;
      for(int i =0;i<corners.size();i++)
        x+=corners[i].x, y+=corners[i].y;
      x/=40;
      y/=40;
      x+=s.x;
      y+=s.y;
      imshow(window_capture_name, frame);
      imshow(window_detection_name, croppedImage);
    }
    lx=x, ly=y;
    if(abs(x-500)<200 && abs(y-500)<200)
    {
      if(lf!=cf-1)
      {
        srv.request.d = true;
        if (client.call(srv))
        {
          ROS_INFO("out: %s", srv.response.out.data());
        }
        else
        {
          ROS_ERROR("Failed to call service danger_region");
          return 1;
        }
      }
      lf=cf;
    }

    char c=(char)waitKey(25);

    msg.X = x;
    msg.Y = y;
    
    ROS_INFO("%d %d %d", msg.X, msg.Y);
// %EndTag(ROSCONSOLE)%

   
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  cap.release();
  destroyAllWindows();


  return 0;
}
// %EndTag(FULLTEXT)%