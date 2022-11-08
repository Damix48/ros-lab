#include <ros/ros.h>
#include <tutorial_01_message/battery_level.h>

void messageCallback(const tutorial_01_message::battery_level::ConstPtr& msg) {
  ROS_INFO("Received battery level:\nID room: %d\nName room: %s\nBattery Level: %f", msg->room_id, msg->room_name.c_str(), msg->battery_level);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "charging_station");
  ros::NodeHandle nodeHandle;
  ros::Subscriber subscriber = nodeHandle.subscribe("charge_status",
                                                    1000, messageCallback);
  ros::spin();
  return 0;
}