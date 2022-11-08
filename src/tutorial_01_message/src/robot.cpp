#include <ros/ros.h>
#include <tutorial_01_message/battery_level.h>

int main(int argc, char **argv) {
  std::map<int, std::string> rooms = {{1, "Robot Vision Lab"},
                                      {2, "SSL Lab"},
                                      {3, "Neurorobotics Lab"},
                                      {4, "IAS-Lab"},
                                      {5, "Autonomous Robotics Lab"}};

  int room = 1;

  ros::init(argc, argv, "robot");

  ros::NodeHandle nodeHandle;

  ros::Publisher publisher = nodeHandle.advertise<tutorial_01_message::battery_level>("charge_status", 1);

  ros::Rate loop_rate(5);

  while (ros::ok()) {
    tutorial_01_message::battery_level batteryLevelMsg;
    batteryLevelMsg.room_id = room;
    batteryLevelMsg.room_name = rooms[room];
    batteryLevelMsg.battery_level = 0.8;

    publisher.publish(batteryLevelMsg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  /* code */
  return 0;
}
