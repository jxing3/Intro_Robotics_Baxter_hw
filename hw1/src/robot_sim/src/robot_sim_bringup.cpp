#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include "robot_sim/joint_state_publisher.h"
#include "robot_sim/robot.h"

void setTargetValues(boost::shared_ptr<robot_sim::Robot> robot)
{
  static bool zeros = false;
  std::vector<double> vals;
  if (zeros) vals.resize(7, 0.0);
  else vals.resize(7, 1.0);
  robot->setTargetValues(vals);
  zeros = !zeros;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_sim");

  boost::shared_ptr<robot_sim::Robot> robot(new robot_sim::Robot(7));
  robot->init();
  boost::shared_ptr<robot_sim::JointStatePublisher> publisher(new robot_sim::JointStatePublisher(robot));
  if (!publisher->init())
  {
    ROS_ERROR("Failed to initialize publisher");
    return 0;
  }

  ros::NodeHandle root_nh("");
  ros::Timer timer = root_nh.createTimer(ros::Duration(1.0), boost::bind(setTargetValues, robot));

  ROS_INFO("Simulated robot spinning");
  ros::spin();					    
}
