#include <apriltag_ros/AprilTagDetectionArray.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

tf2_ros::Buffer buffer;
geometry_msgs::TransformStamped transformmm;

void callbackkk(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg) {
  for (size_t i = 0; i < msg->detections.size(); i++) {
    geometry_msgs::PoseWithCovarianceStamped original(msg->detections[i].pose);
    geometry_msgs::PoseWithCovarianceStamped transformed;

    ROS_INFO("original");
    ROS_INFO_STREAM(original);

    transformed = buffer.transform(original, "base_link", ros::Time(0), msg->detections[i].pose.header.frame_id);

    ROS_INFO("tranformed");
    ROS_INFO_STREAM(transformed);
  }
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "nodeee");

  ros::NodeHandle node;
  ros::Subscriber tag_subscriber = node.subscribe("tag_detections", 1000, callbackkk);

  tf2_ros::TransformListener listener(buffer);

  while (buffer.canTransform("base_link", "camera_base", ros::Time(0))) {
    ros::Duration(0.5).sleep();
  }

  ros::spin();
  return 0;
}
