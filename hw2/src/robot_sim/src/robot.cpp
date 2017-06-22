#include "robot_sim/robot.h"

namespace robot_sim {

Robot::Robot(size_t num_joints) :
  num_joints_(num_joints)
{
  joint_values_.resize(num_joints_, 0.0);
  goal_values_.resize(num_joints_, 0.0);
  max_jump_ = 1.0e-3;
}

Robot::Robot(const std::vector<double> &joint_values)
{
  num_joints_ = joint_values.size();
  joint_values_ = joint_values;
  goal_values_ = joint_values;
  max_jump_ = 1.0e3;
}

bool Robot::init()
{
  running_ = true;
  velocity_mode_ = false;
  last_tick_ = ros::Time::now();
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
  assert(vals.size() == joint_values_.size());
  mutex_.lock();
  goal_values_ = vals;
  velocity_mode_ = false;
  mutex_.unlock(); 
}

void Robot::setVelocities(const std::vector<double> &vels)
{
  assert(vels.size() == joint_values_.size());
  mutex_.lock();
  joint_velocities_ = vels;
  velocity_mode_ = true;
  mutex_.unlock();
}

void Robot::update()
{
  while(running_)
  {
    ros::Time now_tick = ros::Time::now();
    mutex_.lock();
    if (velocity_mode_)
    {
      double secs = (now_tick - last_tick_).toSec();
      for (size_t i=0; i<num_joints_; i++)
      {
	joint_values_[i] += secs * joint_velocities_[i];
      }
    }
    else
    {
      for (size_t i=0; i<num_joints_; i++)
      {
	double jump = goal_values_[i] - joint_values_[i];
	jump = std::max(jump, -max_jump_);
	jump = std::min(jump,  max_jump_);
	joint_values_[i] += jump;
      }
    }
    mutex_.unlock();
    last_tick_ = now_tick;
    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
  }
}

} //namespace robot_sim
