
// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "ark_task_1/board_pose.h"
#include "ark_task_1/danger_region.h"
#include <sstream>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
void chatterCallback(const ark_task_1::board_pose::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d] [%d]", msg->X, msg->Y);
}

bool service_call(ark_task_1::danger_region::Request  &req,
          ark_task_1::danger_region::Response &res)
{
 std::stringstream ss;
  ss << "hello world ";
  res.out = ss.str(); 
  ROS_INFO("request: x=%d", (int)req.d);
  ROS_INFO("sending back response: [%s]", res.out.data());
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("danger_region", service_call);

  ros::Subscriber sub = n.subscribe("check_pose", 1000, chatterCallback);

  ros::spin();
  return 0;
}
