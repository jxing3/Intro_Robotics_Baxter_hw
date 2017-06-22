#pragma once

#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <urdf/model.h>

#include "robot_sim/robot.h"

namespace robot_sim {

class JointStatePublisher 
{
 public:
 JointStatePublisher(boost::shared_ptr<Robot> robot) : 
  root_nh_(""), robot_(robot)
  {}

  bool init();

 private:
  ros::NodeHandle root_nh_;
  boost::shared_ptr<Robot> robot_;
  ros::Publisher state_publisher_;
  ros::Timer state_timer_;
  urdf::Model model_;
  std::vector<std::string> joint_names_;
  
  void callback(const ros::TimerEvent&);
};

} //namespace robot_sim
