#include <salih_marangoz_thesis/robot_monitor.h>

namespace salih_marangoz_thesis
{

// const JointLinkStateConstPtr
// RobotMonitor::getJointLinkState()
// {
//   async_mtx_.lock();
//   JointLinkStatePtr tmp = async_state_;
//   async_mtx_.unlock();
//   return tmp;
// }

RobotMonitor::RobotMonitor(ros::NodeHandle &nh, ros::NodeHandle &priv_nh): nh(nh), priv_nh(priv_nh)
{
  joint_state_sub = nh.subscribe("/joint_states", 2, &RobotMonitor::jointStateCallback, this);
  link_state_thread = new boost::thread(boost::bind(&RobotMonitor::updateLinkThread, this));
  collision_state_thread = new boost::thread(boost::bind(&RobotMonitor::updateCollisionThread, this));
  visualization_thread = new boost::thread(boost::bind(&RobotMonitor::updateVisualizationThread, this));
}

RobotMonitor::~RobotMonitor()
{
  link_state_thread->join();
  delete link_state_thread;
  collision_state_thread->join();
  delete collision_state_thread;
}

void
RobotMonitor::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  ROS_INFO_ONCE("RobotMonitor: Received joint state!");
  last_joint_state_mtx.lock();
  last_joint_state_msg = msg;
  last_joint_state_mtx.unlock();

  for (int i=0; i<robot::num_joints; i++)
  { 
    ptrdiff_t pos = find(msg->name.begin(), msg->name.end(), robot::joint_names[i]) - msg->name.begin();
    if(pos < msg->name.size())
    {
      joint_idx_to_msg_idx[i] = pos; // i->joint_idx, pos->msg_idx
    }
    else
    {
      joint_idx_to_msg_idx[i] = -1;
    }
  }
}

void
RobotMonitor::updateLinkThread()
{
  ros::Rate r(100.0); // TODO
  while (ros::ok())
  {
    if (last_joint_state_msg == nullptr){r.sleep(); continue;}

    last_joint_state_mtx.lock();
    sensor_msgs::JointStateConstPtr cached_joint_state_msg = last_joint_state_msg;
    last_joint_state_mtx.unlock();
    
    JointLinkStatePtr new_state = computeJointLinkState(cached_joint_state_msg);
    if (new_state == nullptr){r.sleep(); continue;}

    last_joint_link_state_mtx.lock();
    last_joint_link_state_msg = new_state;
    last_joint_link_state_mtx.unlock();
  
    ROS_INFO_ONCE("RobotMonitor: Computed link state!");
    r.sleep();
  }
}

void
RobotMonitor::updateCollisionThread()
{
  ros::Rate r(100.0); // TODO
  while (ros::ok())
  {
    if (last_joint_link_state_msg == nullptr){r.sleep(); continue;}

    last_joint_link_state_mtx.lock();
    JointLinkStateConstPtr cached_joint_link_state_msg = last_joint_link_state_msg;
    last_joint_link_state_mtx.unlock();
    
    JointLinkCollisionStatePtr new_state = computeJointLinkCollisionState(cached_joint_link_state_msg);
    if (new_state == nullptr){r.sleep(); continue;}

    last_joint_link_collision_state_mtx.lock();
    last_joint_link_collision_state_msg = new_state;
    last_joint_link_collision_state_mtx.unlock();

    ROS_INFO_ONCE("RobotMonitor: Computed collision state!");
    r.sleep();
  }
}

void
RobotMonitor::updateVisualizationThread()
{
  ros::Rate r(60.0); // 60fps is more than enough!
  while (ros::ok())
  {
    if (last_joint_link_collision_state_msg == nullptr){r.sleep(); continue;}

    last_joint_link_state_mtx.lock();
    JointLinkCollisionStateConstPtr cached_joint_link_collision_state_mtx = last_joint_link_collision_state_msg;
    last_joint_link_state_mtx.unlock();
    
    computeAndPublishVisualization(cached_joint_link_collision_state_mtx);

    ROS_INFO_ONCE("RobotMonitor: Published visualization!");
    r.sleep();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

JointLinkStatePtr
RobotMonitor::computeJointLinkState(const sensor_msgs::JointStateConstPtr& msg)
{
  JointLinkStatePtr state = boost::make_shared<JointLinkState>();
  state->joint_state = *msg; // copy
  state->header = msg->header; // copy

  // recompute kinematic chain
  std::vector<double> &global_link_transformations = state->link_state.transformations; // copy reference
  global_link_transformations.resize(robot::num_links*7);
  for (int i=0; i<robot::num_joints; i++)
  {
    int child_link_idx = robot::joint_child_link_idx[i];
    int parent_link_idx = robot::joint_parent_link_idx[i];
    int msg_idx = joint_idx_to_msg_idx[i];

    // init
    if (parent_link_idx == -1)
    {
      global_link_transformations[7*child_link_idx+0] = 0.0;
      global_link_transformations[7*child_link_idx+1] = 0.0;
      global_link_transformations[7*child_link_idx+2] = 0.0;
      global_link_transformations[7*child_link_idx+3] = 1.0;
      global_link_transformations[7*child_link_idx+4] = 0.0;
      global_link_transformations[7*child_link_idx+5] = 0.0;
      global_link_transformations[7*child_link_idx+6] = 0.0;
      continue;
    }

    // Translation
    if (robot::link_can_skip_translation[child_link_idx])
    {
        global_link_transformations[7*child_link_idx+0] = global_link_transformations[7*parent_link_idx+0];
        global_link_transformations[7*child_link_idx+1] = global_link_transformations[7*parent_link_idx+1];
        global_link_transformations[7*child_link_idx+2] = global_link_transformations[7*parent_link_idx+2];
    }
    else
    {
      utils::computeLinkTranslation(&(global_link_transformations[7*parent_link_idx]), // translation
                                    &(global_link_transformations[7*parent_link_idx+3]),  // rotation
                                    &(robot::link_transform_translation_only[3*child_link_idx][0]), 
                                    &(global_link_transformations[7*child_link_idx]));
    }

    if (msg_idx!=-1) // if joint can move
    { 
      double joint_val = state->joint_state.position[msg_idx];
      
      if (robot::link_can_skip_rotation[child_link_idx]) // if can skip the rotation then only rotate using the joint position
      {
        utils::computeLinkRotation(&(global_link_transformations[7*parent_link_idx+3]),
                                  joint_val, 
                                  &(global_link_transformations[7*child_link_idx+3]));
      }
      else // if link has rotation and joint has rotation, then we need to rotate using both
      {
        utils::computeLinkRotation(&(global_link_transformations[7*parent_link_idx+3]), 
                                  &(robot::link_transform_quaternion_only[4*child_link_idx][0]), 
                                  joint_val, 
                                  &(global_link_transformations[7*child_link_idx+3]));
      }
    }
    else // if joint is static
    {
      if (robot::link_can_skip_rotation[child_link_idx]) // if can skip the rotation then no need to do anything
      {
        global_link_transformations[7*child_link_idx+3] = global_link_transformations[7*parent_link_idx+3];
        global_link_transformations[7*child_link_idx+4] = global_link_transformations[7*parent_link_idx+4];
        global_link_transformations[7*child_link_idx+5] = global_link_transformations[7*parent_link_idx+5];
        global_link_transformations[7*child_link_idx+6] = global_link_transformations[7*parent_link_idx+6];
      }
      else // if link has a rotation, only compute that
      {
        utils::computeLinkRotation(&(global_link_transformations[7*parent_link_idx+3]), // global parent rotation
                                  &(robot::link_transform_quaternion_only[4*child_link_idx][0]), // local child rotation
                                  &(global_link_transformations[7*child_link_idx+3]));  // global child rotation
      }

    }
  }

  return state;
}

JointLinkCollisionStatePtr
RobotMonitor::computeJointLinkCollisionState(const JointLinkStateConstPtr& msg)
{
  std::scoped_lock lock_collision_managers(int_collision_manager_mtx, ext_collision_manager_mtx);
  
  JointLinkCollisionStatePtr state = boost::make_shared<JointLinkCollisionState>();
  state->joint_state = *msg; // copy
  state->header = msg->header; // copy

  if (int_collision_objects.size() <= 0) int_collision_objects = robot::getRobotCollisionObjects(); // init int_collision_objects if not initialized

  // update internal collision objects
  // TODO: optimize using these, maybe? robot::object_can_skip_rotation[i]; ;
  for (int object_idx=0; object_idx<robot::num_objects; object_idx++)
  {
    int link_idx = robot::object_idx_to_link_idx[object_idx];
    bool can_skip_translation = robot::object_can_skip_translation[object_idx];
    bool can_skip_rotation = robot::object_can_skip_rotation[object_idx];
    const double* link_translation = &(msg->link_state.transformations[7*link_idx]);
    const double* link_rotation = &(msg->link_state.transformations[7*link_idx+3]);
    const double* object_translation = &(robot::object_transform_translation_only[object_idx][0]);
    const double* object_rotation = &(robot::object_transform_quaternion_only[object_idx][0]);

    double final_object_translation[3];
    double final_object_rotation[3];

    robot::object_transform_translation_only[object_idx];

    // compute translation
    if (robot::object_can_skip_translation[object_idx])
    {
      final_object_translation[0] = link_translation[0];
      final_object_translation[1] = link_translation[1];
      final_object_translation[2] = link_translation[2];
    }
    else
    {
      utils::computeLinkTranslation(link_translation, link_rotation, object_translation, final_object_translation);
    }

    // compute rotation
    if (robot::object_can_skip_rotation[object_idx])
    {
      final_object_rotation[3] = link_translation[3];
      final_object_rotation[4] = link_translation[4];
      final_object_rotation[5] = link_translation[5];
      final_object_rotation[6] = link_translation[6];
    }
    else
    {
      utils::computeLinkRotation(link_rotation, object_rotation, final_object_rotation);
    }

    int_collision_objects[object_idx]->setTranslation(Vec3f(final_object_translation[0], final_object_translation[1], final_object_translation[2]));
    int_collision_objects[object_idx]->setRotation(Eigen::Quaterniond(final_object_rotation).toRotationMatrix());
    int_collision_objects[object_idx]->computeAABB();
  }

  if (int_collision_manager.size() <= 0) int_collision_manager.registerObjects(int_collision_objects); // init collision manager if not initialized
  int_collision_manager.update();


  // TODO: handle external objects

  return state;
}

void
RobotMonitor::computeAndPublishVisualization(const JointLinkCollisionStateConstPtr& msg)
{
  visualization_msgs::MarkerArray arr;

  // Visualize robot collision body
  
  for (int i=0; i<utils::countOf(robot::collisions); i++)
  {
    visualization_msgs::Marker marker;
    const robot::Collision &obj = robot::collisions[i];

    double obj_pos[3];
    double result[3];
    obj_pos[0] = obj.x;
    obj_pos[1] = obj.y;
    obj_pos[2] = obj.z;
    utils::computeLinkTranslation(&(glt[7*obj.link_idx]),
                                  &(glt[7*obj.link_idx+3]),
                                  obj_pos,
                                  result);

    marker.header = msg->header;
    marker.ns = "collisions";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = result[0];
    marker.pose.position.y = result[1];
    marker.pose.position.z = result[2];
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = obj.radius*2;
    marker.scale.y = obj.radius*2;
    marker.scale.z = obj.radius*2;
    marker.color.a = 0.75; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    arr.markers.push_back(marker);
  }
  marker_array_pub.publish(arr);
}



} // namespace salih_marangoz_thesis