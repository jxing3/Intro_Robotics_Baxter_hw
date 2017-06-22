#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include "robot_sim/joint_state_publisher.h"
#include "robot_sim/robot.h"
#include "robot_sim/velocity_controller.h"

void setTargetValues(boost::shared_ptr<robot_sim::Robot> robot, int num_joints)
{
  static bool zeros = false;
  std::vector<double> vals;
  if (zeros) vals.resize(num_joints, 0.0);
  else vals.resize(num_joints, 1.0);
  robot->setTargetValues(vals);
  zeros = !zeros;
}

void setVelocities(boost::shared_ptr<robot_sim::Robot> robot, int num_joints)
{
  static int count = 0;
  std::vector<double> vals;
  if (count == 0) vals.resize(num_joints, 1.0);
  else if (count == 1) vals.resize(num_joints, 0.0);
  else if (count == 2) vals.resize(num_joints, -1.0);
  else if (count == 3) vals.resize(num_joints, 0.0);
  robot->setVelocities(vals);
  if (++count > 3) count = 0; 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_sim");

  ros::NodeHandle priv_nh("~");
  int num_joints;
  priv_nh.param<int>("num_joints", num_joints, 7);
  //get Kuka robot out of singularity
  std::vector<double> joints(num_joints, 0.0);
  joints[1] = 0.85;
  joints[3] = 0.85;
  boost::shared_ptr<robot_sim::Robot> robot(new robot_sim::Robot(joints));
  robot->init();
  boost::shared_ptr<robot_sim::JointStatePublisher> publisher(
						new robot_sim::JointStatePublisher(robot));
  if (!publisher->init())
  {
    ROS_ERROR("Failed to initialize publisher");
    return 0;
  }
  boost::shared_ptr<robot_sim::VelocityController> controller(
						new robot_sim::VelocityController(robot));
  if (!controller->init())
  {
    ROS_ERROR("Failed to initialize controller");
    return 0;
  }

  ros::NodeHandle root_nh("");
  //ros::Timer timer = root_nh.createTimer(ros::Duration(1.0), boost::bind(setVelocities, robot, num_joints));

  ROS_INFO("Simulated robot spinning");
  ros::spin();					    
}
