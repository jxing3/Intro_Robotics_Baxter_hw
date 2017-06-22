#pragma once

#include <boost/thread.hpp>
#include <vector>

#include <ros/ros.h>

namespace robot_sim {

class Robot
{
 public:
  Robot(size_t num_joints);
  bool init();
  void stop();

  std::vector<double> getJointValues() const;
  void setTargetValues(const std::vector<double> &vals);

 private:
  size_t num_joints_;
  std::vector<double> joint_values_;
  std::vector<double> goal_values_;
  double max_jump_;
  boost::thread thread_;
  mutable boost::mutex mutex_;
  bool running_;

  void update();
};

} //namespace robot_sim
