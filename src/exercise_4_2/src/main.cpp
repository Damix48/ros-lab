#include <apriltag_ros/AprilTagDetectionArray.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

class Boh {
 private:
  ros::NodeHandle node;
  ros::Subscriber tag_subscriber;

  std::vector<geometry_msgs::PoseWithCovarianceStamped> points;

  cv::Point3f meanPoint;
  cv::Point2f meanPoint2D;

  std::string saveImagePath;
  cv::Mat image;

  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;

  void callback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg) {
    ROS_INFO("Detected %d tags", msg->detections.size());

    for (size_t i = 0; i < msg->detections.size(); i++) {
      geometry_msgs::PoseWithCovarianceStamped original(msg->detections[i].pose);

      if (points.size() == 0 || original.header.seq != points.at(0).header.seq) {
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

    std::vector<float> tempK;
    for (size_t i = 0; i < cameraInfo->K.size(); i++) {
      tempK.push_back(cameraInfo->K[i]);
    }
    cameraMatrix = cv::Mat(3, 3, CV_32F, tempK.data()).clone();

    std::vector<float> tempD;
    for (size_t i = 0; i < cameraInfo->D.size(); i++) {
      tempD.push_back(cameraInfo->D[i]);
    }
    distCoeffs = cv::Mat(1, 8, CV_32F, tempD.data()).clone();
  }

  void processPoints() {
    ROS_INFO("There are %d points", points.size());
    for (size_t i = 0; i < points.size(); i++) {
      meanPoint.x += points[i].pose.pose.position.x;
      meanPoint.y += points[i].pose.pose.position.y;
      meanPoint.z += points[i].pose.pose.position.z;
    }
    meanPoint = meanPoint / (float)points.size();

    ROS_INFO("The mean point is (%f, %f, %f)", meanPoint.x, meanPoint.y, meanPoint.z);
  }

  void projectMean() {
    cv::Mat rvec = (cv::Mat_<float>(3, 1) << 0.0, 0.0, 0.0);
    cv::Mat tvec = (cv::Mat_<float>(3, 1) << meanPoint.x, meanPoint.y, meanPoint.z);

    std::vector<cv::Point3f> tempPoints;
    tempPoints.push_back(meanPoint);
    std::vector<cv::Point2f> tempPoints2D;

    cv::projectPoints(tempPoints, rvec, tvec, cameraMatrix, distCoeffs, tempPoints2D);

    meanPoint2D = tempPoints2D[0];
  }

  void drawMean() {
    cv::circle(image, meanPoint2D, 3, cv::Scalar(255, 0, 0));
    cv::imwrite(saveImagePath, image);
  }

 public:
  Boh(std::string imagePath) : meanPoint(0, 0, 0), node(ros::NodeHandle()) {
    image = cv::imread(imagePath);
    saveImagePath = imagePath + ".saved.png";

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
