#include <ros/ros.h>
#include <tutorial_02_service/battery_level.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "charging_station");

  ros::NodeHandle nodeHandle;
  ros::ServiceClient serviceClient = nodeHandle.serviceClient<tutorial_02_service::battery_level>("charge_status");

  int id = atoi(argv[1]);
  int frequency = atoi(argv[2]);

  ros::Rate loop_rate(frequency);

  while (ros::ok()) {
    tutorial_02_service::battery_level service;
    service.request.station_id = id;

    if (serviceClient.call(service)) {
      ROS_INFO("ID: %i", id);
      ROS_INFO("Received battery level:\nID room: %d\nName room: %s\nBattery Level: %f", service.response.battery_level.room_id, service.response.battery_level.room_name.c_str(), service.response.battery_level.battery_level);
    } else {
      ROS_ERROR("Error");
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}