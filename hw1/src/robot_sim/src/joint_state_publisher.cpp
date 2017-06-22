#include "robot_sim/joint_state_publisher.h"

#include <sensor_msgs/JointState.h>

namespace robot_sim {

bool JointStatePublisher::init()
{
  if (!robot_) return false;

  state_publisher_ = root_nh_.advertise<sensor_msgs::JointState>("joint_states", 10);
  state_timer_ = root_nh_.createTimer(ros::Duration(0.1), &JointStatePublisher::callback, this);

  if (!model_.initParam("robot_description"))
  {
    ROS_ERROR("Joint state publlisher: failed to read urdf from parameter server");
    return false;
  }
  for (std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it = model_.joints_.begin();
       it != model_.joints_.end(); it++)
  {
    if (it->second->type != urdf::Joint::FIXED)
    {
      joint_names_.push_back(it->first);
    }
  }

  ROS_INFO_STREAM("Robot model read with " << joint_names_.size() << " non-fixed joints.");

  return true;
}

void JointStatePublisher::callback(const ros::TimerEvent&)
{
  assert(robot_);
  sensor_msgs::JointState msg;
  msg.position = robot_->getJointValues();
  msg.name = joint_names_;
  if (msg.position.size() != msg.name.size())
  {
    ROS_ERROR("Mismatch between sim robot and joint state publisher");
    return;
  }
  msg.header.stamp = ros::Time::now();
  state_publisher_.publish(msg);
}

}
