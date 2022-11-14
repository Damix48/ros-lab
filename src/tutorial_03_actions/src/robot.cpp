#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <tutorial_03_actions/batteryAction.h>

class BatteryAction {
 protected:
  ros::NodeHandle nodeHandle;
  actionlib::SimpleActionServer<tutorial_03_actions::batteryAction> actionServer;

  float batteryLevel;

 public:
  BatteryAction(std::string topic, float batteryLevel_) : actionServer(nodeHandle, topic, boost::bind(&BatteryAction::charging, this, _1), false),
                                                          batteryLevel(batteryLevel_) {
    actionServer.start();
  }

  void charging(const tutorial_03_actions::batteryGoalConstPtr& goal) {
    tutorial_03_actions::batteryResult result;
    tutorial_03_actions::batteryFeedback feedback;

    int rate_ = 1;

    ros::Rate rate(rate_);
    ros::Duration chargingTime(60);

    ros::Time startingTime = ros::Time::now();

    float chargingRate = (goal->target_battery - batteryLevel) / (chargingTime.sec * rate_);

    while (ros::Time::now() - startingTime < chargingTime) {
      batteryLevel += chargingRate;

      feedback.current_battery = batteryLevel;

      actionServer.publishFeedback(feedback);

      rate.sleep();
    }

    batteryLevel = goal->target_battery;

    if (batteryLevel == goal->target_battery) {
      result.reached = true;

      actionServer.setSucceeded(result);
    } else {
      result.reached = false;

      actionServer.setSucceeded(result);
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot");

  BatteryAction action("charging", 0.05);

  ros::spin();

  return 0;
}