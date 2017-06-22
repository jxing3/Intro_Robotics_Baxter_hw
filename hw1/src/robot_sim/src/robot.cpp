#include "robot_sim/robot.h"

namespace robot_sim {

Robot::Robot(size_t num_joints) :
  num_joints_(num_joints)
{
  joint_values_.resize(num_joints_, 0.0);
  goal_values_.resize(num_joints_, 0.0);
  max_jump_ = 1.0e-2;
}

bool Robot::init()
{
  running_ = true;
  thread_ = boost::thread(boost::bind(&Robot::update, this));
  return true;
}

void Robot::stop()
{
  running_ = false;
  thread_.join();
}

std::vector<double> Robot::getJointValues() const
{
  mutex_.lock();
  std::vector<double> vals = joint_values_;
  mutex_.unlock();
  return vals;
}

void Robot::setTargetValues(const std::vector<double> &vals)
{
  mutex_.lock();
  goal_values_ = vals;
  mutex_.unlock(); 
}

void Robot::update()
{
  while(running_)
  {
    mutex_.lock();
    for (size_t i=0; i<num_joints_; i++)
    {
      double jump = goal_values_[i] - joint_values_[i];
      jump = std::max(jump, -max_jump_);
      jump = std::min(jump,  max_jump_);
      joint_values_[i] += jump;
    }
    mutex_.unlock();
    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
  }
}

} //namespace robot_sim
