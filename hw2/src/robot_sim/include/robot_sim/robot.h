#pragma once

#include <boost/thread.hpp>
#include <vector>

#include <ros/ros.h>

namespace robot_sim {

class Robot
{
 public:
  Robot(size_t num_joints);
  Robot(const std::vector<double> &joint_values);
  bool init();
  void stop();

  size_t getNumJoints() const {return num_joints_;}

  std::vector<double> getJointValues() const;
  void setTargetValues(const std::vector<double> &vals);

  void setVelocities(const std::vector<double> &vels);

 private:
  size_t num_joints_;
  std::vector<double> joint_values_;
  std::vector<double> goal_values_;
  std::vector<double> joint_velocities_;
  bool velocity_mode_;
  double max_jump_;
  boost::thread thread_;
  mutable boost::mutex mutex_;
  bool running_;
  ros::Time last_tick_;

  void update();
};

} //namespace robot_sim
