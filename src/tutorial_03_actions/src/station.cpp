#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <tutorial_03_actions/batteryAction.h>

void doneCallback(const actionlib::SimpleClientGoalState& state,
                  const tutorial_03_actions::batteryResultConstPtr& result) {
  ROS_INFO("Finished in state [%s]", state.toString().c_str());

  if (result->reached) {
    ROS_INFO("Goal reached");
  } else {
    ROS_ERROR("Goal not reached");
  }

  ros::shutdown();
}

void feedbackCallback(const tutorial_03_actions::batteryFeedbackConstPtr& feeback) {
  ROS_INFO("Battery level: %f", feeback->current_battery);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "charging_station");

  actionlib::SimpleActionClient<tutorial_03_actions::batteryAction> actionClient("charging", true);

  ROS_INFO("Waiting for action server to start.");
  actionClient.waitForServer();
  ROS_INFO("Action server started, sending goal.");

  tutorial_03_actions::batteryGoal goal;
  goal.target_battery = 0.8;

  actionClient.sendGoal(goal,
                        &doneCallback,
                        actionlib::SimpleActionClient<tutorial_03_actions::batteryAction>::SimpleActiveCallback(),
                        &feedbackCallback);

  bool timeout = actionClient.waitForResult(ros::Duration(120));

  if (!timeout) {
    ROS_INFO("Exiting because the action not finished before the timeout");
  }

  return 0;
}