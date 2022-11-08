#include <ros/ros.h>
#include <tutorial_02_service/battery_level.h>

bool batteryLevelServiceResponse(tutorial_02_service::battery_level::Request& request, tutorial_02_service::battery_level::Response& response) {
  std::map<int, std::string> rooms = {{1, "Robot Vision Lab"},
                                      {2, "SSL Lab"},
                                      {3, "Neurorobotics Lab"},
                                      {4, "IAS-Lab"},
                                      {5, "Autonomous Robotics Lab"}};

  int room = 1;

  response.battery_level.room_id = room;
  response.battery_level.room_name = rooms[room];
  response.battery_level.battery_level = 0.85;

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot");

  ros::NodeHandle nodeHandle;
  ros::ServiceServer serviceServer = nodeHandle.advertiseService("charge_status", batteryLevelServiceResponse);

  ros::spin();

  return 0;
}
