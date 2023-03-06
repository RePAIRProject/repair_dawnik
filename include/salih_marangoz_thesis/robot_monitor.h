#ifndef __ROBOT_MONITOR_H__
#define __ROBOT_MONITOR_H__

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <algorithm> // find
#include <vector>
#include <string>
#include <boost/thread.hpp> // TODO: use std?
#include <mutex>

#include <salih_marangoz_thesis/utils.h>


#ifndef ROBOT_MONITOR_NO_DEFAULTS
  #include <salih_marangoz_thesis/robot_configuration/robot_configuration.h>
  #define ROBOT_MON_DEF_joint_states_topic = std::string("/joint_states")
  #define ROBOT_MON_DEF_global_link_transformations_async_thread_rate = 100.0
  #define ROBOT_MON_DEF_num_joints = robot::num_joints
  #define ROBOT_MON_DEF_num_links = robot::num_links
  #define ROBOT_MON_DEF_joint_names = robot::joint_names
  #define ROBOT_MON_DEF_joint_child_link_idx = robot::joint_child_link_idx
  #define ROBOT_MON_DEF_joint_parent_link_idx = robot::joint_parent_link_idx
  #define ROBOT_MON_DEF_link_transform_translation_only = (double*)robot::link_transform_translation_only // knowing that 2D C arrays are contiguous
  #define ROBOT_MON_DEF_link_transform_quaternion_only = (double*)robot::link_transform_quaternion_only // knowing that 2D C arrays are contiguous
  #define ROBOT_MON_DEF_link_can_skip_translation = robot::link_can_skip_translation
  #define ROBOT_MON_DEF_link_can_skip_rotation = robot::link_can_skip_rotation
#endif

namespace salih_marangoz_thesis
{

class RobotMonitor
{
public:
  RobotMonitor(ros::NodeHandle &nh,
               const std::string joint_states_topic ROBOT_MON_DEF_joint_states_topic,
               double global_link_transformations_async_thread_rate ROBOT_MON_DEF_global_link_transformations_async_thread_rate,
               int num_joints ROBOT_MON_DEF_num_joints, 
               int num_links ROBOT_MON_DEF_num_links,
               const std::string* joint_names ROBOT_MON_DEF_joint_names,
               const int* joint_child_link_idx ROBOT_MON_DEF_joint_child_link_idx,
               const int* joint_parent_link_idx ROBOT_MON_DEF_joint_parent_link_idx,
               const double* link_transform_translation_only ROBOT_MON_DEF_link_transform_translation_only,
               const double* link_transform_quaternion_only ROBOT_MON_DEF_link_transform_quaternion_only,
               const int* link_can_skip_translation ROBOT_MON_DEF_link_can_skip_translation,
               const int* link_can_skip_rotation ROBOT_MON_DEF_link_can_skip_rotation
               );
  ~RobotMonitor();

  // x1,y1,z1,rw1,rx1,ry1,rz1,x2,y2,z2,rw2,rx2,ry2,rz2,... (linear1,quaternion1,linear2,quaternion2,...)
  // return_cached_transformations=true to skip updating the kinematic tree
  // // If you used getGlobalLinkTransformationsAsync at least once, do not use getGlobalLinkTransformations again!
  const std::shared_ptr<std::vector<double>> getGlobalLinkTransformations();
  const std::shared_ptr<std::vector<double>> getGlobalLinkTransformationsUnsafe();
  const std::shared_ptr<std::vector<double>> getGlobalLinkTransformationsAsync();

  // setter/getter
  int getNumJoints(){return num_joints_;}
  int getNumLinks(){return num_links_;}
  const std::string* getJointNames(){return joint_names_;}
  int getChildLinkIdx(int joint_idx){return joint_child_link_idx_[joint_idx];}
  int getParentLinkIdx(int joint_idx){return joint_parent_link_idx_[joint_idx];}
  int getMessageIdx(int joint_idx){return joint_idx_to_msg_idx_[joint_idx];}
  const sensor_msgs::JointStateConstPtr getJointStateMsg()
  {
    msg_mtx_.lock();
    const sensor_msgs::JointStateConstPtr tmp = last_joint_state_msg_;
    msg_mtx_.unlock();
    return tmp;
  }

public:
  void jointStateCallback_(const sensor_msgs::JointStateConstPtr& msg);
  void globalLinkTransformationsAsyncThread_();
private:
  bool computeGlobalLinkTransformations_(std::vector<double> &global_link_transformations);
private:
  // from the constructor
  ros::NodeHandle nh_;
  const std::string joint_states_topic_;
  double global_link_transformations_async_thread_rate_;
  int num_joints_;
  int num_links_;
  const std::string* joint_names_; // len: num_joints_
  const int* joint_child_link_idx_; // len: num_joints_
  const int* joint_parent_link_idx_; // len: num_joints_
  const double* link_transform_translation_only_; // len: num_links_*3
  const double* link_transform_quaternion_only_; // len: num_links_*4
  const int* link_can_skip_translation_; // len: num_links_
  const int* link_can_skip_rotation_; // len: num_links_
  // internal
  std::mutex msg_mtx_;
  sensor_msgs::JointStateConstPtr last_joint_state_msg_;
  bool joint_state_cache_is_dirty_;
  bool joint_state_async_is_dirty_;
  //std::mutex async_cache_; // cache is used internally, and not overriden. so not needed
  std::shared_ptr<std::vector<double>> global_link_transformations_cache_; // len: num_links_*7 layout: x1,y1,z1,rw1,rx1,ry1,rz1,x2,y2,z2,rw2,rx2,ry2,rz2,... (linear1,quaternion1,linear2,quaternion2,...)
  std::mutex async_mtx_;
  std::shared_ptr<std::vector<double>> global_link_transformations_async_; // len: num_links_*7 layout: x1,y1,z1,rw1,rx1,ry1,rz1,x2,y2,z2,rw2,rx2,ry2,rz2,... (linear1,quaternion1,linear2,quaternion2,...)
  int* joint_idx_to_msg_idx_; // len: num_joints_
  boost::thread* global_link_transformations_async_thread_;
  ros::Subscriber joint_states_sub_;

};

} // namespace salih_marangoz_thesis















#endif // __ROBOT_MONITOR_H__