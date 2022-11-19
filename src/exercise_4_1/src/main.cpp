#include <apriltag_ros/AprilTagDetectionArray.h>
#include <ros/ros.h>

void callbackkk(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg) {
  ROS_INFO("ciao");
  ROS_INFO("%d", msg->detections.size());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "nodeee");

  ros::NodeHandle node;
  ros::Subscriber subscriber = node.subscribe("tag_detections", 1000, callbackkk);

  ROS_INFO("ciao2");

  ros::spin();
  return 0;
}
