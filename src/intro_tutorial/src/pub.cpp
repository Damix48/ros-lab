#include <intro_tutorial/msg1.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "pub_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<intro_tutorial::msg1>("damiano", 1);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    intro_tutorial::msg1 msg;
    msg.A = 1;
    msg.B = 2;
    msg.C = 3;
    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
