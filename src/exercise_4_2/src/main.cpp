#include <apriltag_ros/AprilTagDetectionArray.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

class Boh {
 private:
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener* listener;

  ros::NodeHandle node;
  ros::Subscriber tag_subscriber;

  std::vector<geometry_msgs::PoseWithCovarianceStamped> points;

  cv::Point3d meanPoint;
  cv::Point2d meanPoint2D;

  std::string saveImagePath;
  cv::Mat image;

  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;

  void callback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg) {
    ROS_INFO("Detected %d tags", msg->detections.size());

    for (size_t i = 0; i < msg->detections.size(); i++) {
      geometry_msgs::PoseWithCovarianceStamped original(msg->detections[i].pose);
      // geometry_msgs::PoseWithCovarianceStamped transformed = buffer.transform(original, "base_link", ros::Time(0), original.header.frame_id);
      // transformed.header.seq = original.header.seq;

      // points.push_back(transformed);

      if (points.size() == 0 || original.header.seq != points.at(0).header.seq) {
        // geometry_msgs::PoseWithCovarianceStamped transformed = buffer.transform(original, "base_link", ros::Time(0), original.header.frame_id);

        // transformed.header.seq = original.header.seq;

        points.push_back(original);
      } else {
        ROS_INFO("shutdown");
        tag_subscriber.shutdown();

        loadCameraInfo();
        processPoints();
        projectMean();
        drawMean();
      }
    }
  }

  void loadCameraInfo() {
    boost::shared_ptr<const sensor_msgs::CameraInfo> cameraInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("kinect/rgb/camera_info", ros::Duration(5));

    float tempK[9];
    for (size_t i = 0; i < 9; i++) {
      tempK[i] = cameraInfo->K[i];
    }

    cameraMatrix = cv::Mat(3, 3, CV_32F, tempK);

    distCoeffs = cv::Mat(cameraInfo->D);

    std::cout << cameraMatrix << std::endl;
    std::cout << distCoeffs << std::endl;
  }

  void processPoints() {
    for (size_t i = 0; i < points.size(); i++) {
      std::cout << points[i].pose.pose.position.x << " " << points[i].pose.pose.position.y << " " << points[i].pose.pose.position.z << std::endl;
      meanPoint.x += points[i].pose.pose.position.x;
      meanPoint.y += points[i].pose.pose.position.y;
      meanPoint.z += points[i].pose.pose.position.z;
    }
    std::cout << meanPoint << std::endl;
    meanPoint = meanPoint / (float)points.size();

    ROS_INFO("The mean point is (%f, %f, %f)", meanPoint.x, meanPoint.y, meanPoint.z);
  }

  void projectMean() {
    cv::Mat rvec = (cv::Mat_<float>(3, 1) << 0.0, 0.0, 0.0);
    cv::Mat tvec = (cv::Mat_<float>(3, 1) << meanPoint.x, meanPoint.y, meanPoint.z);
    // cv::Mat tvec(meanPoint);

    std::vector<cv::Point3d> tempPoints({meanPoint});
    std::vector<cv::Point2d> tempPoints2D;

    cv::projectPoints(tempPoints, rvec, tvec, cameraMatrix, distCoeffs, tempPoints2D);

    meanPoint2D = tempPoints2D[0];

    ROS_INFO("The mean point in 2D is (%f, %f)", meanPoint2D.x, meanPoint2D.y);
  }

  void drawMean() {
    cv::circle(image, meanPoint2D, 3, cv::Scalar(255, 0, 0));
    cv::imwrite(saveImagePath, image);
  }

 public:
  Boh(std::string imagePath) : meanPoint(0, 0, 0), node(ros::NodeHandle()) {
    image = cv::imread(imagePath);
    saveImagePath = imagePath + ".saved.png";

    listener = new tf2_ros::TransformListener(buffer, node);
    tag_subscriber = node.subscribe("tag_detections", 1000, &Boh::callback, this);

    ROS_INFO("Init");
  }

  Boh(const Boh& right_side) {}
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "nodeee");

  Boh ciao = Boh(argv[1]);

  ros::spin();
  return 0;
}
