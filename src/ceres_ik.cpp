#include <salih_marangoz_thesis/ceres_ik.h>

/////////////////////////////////////// EXPERIMENTAL /////////////////////////////////////////////////////////////
namespace ceres
{
RelaxedIKLoss::RelaxedIKLoss(double a) : a_(a) {
  CHECK_GE(a, 0.0);
}

// desmos: https://www.desmos.com/calculator/9epwyzkkk7
// based on "RelaxedIK: Real-time Synthesis of Accurate and Feasible Robot Arm Motion" by Daniel Rakita, et. al.
// note: relaxed ^4 to ^2 for the outlier part of the loss. ^4 is crazy...
// more note: relaxed ^2 to l1 for the outlier part. this may be used for something else...
void RelaxedIKLoss::Evaluate(double s, double* rho) const {
    const double c = 0.1; // TODO
    const double r = 0.01; // TODO
    const double s2 = s*s;
    const double c2 = c*c;
    const double c4 = c2*c2;
    const double g = std::exp( -s2/(2*c2) );

    // L2 loss for outliers
    //rho[0] = a_*( -g + r*s2 + 1 );
    //rho[1] = a_*( (s*g) / c2 + 2*r*s );
    //rho[2] = a_*( g/c2 - (s2*g)/c4 + 2*r );

    // L1 loss for outliers
    rho[0] = a_*( -g + r*s + 1 );
    rho[1] = a_*( (s*g) / c2 + r );
    rho[2] = a_*( g/c2 - (s2*g)/c4 );
}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace salih_marangoz_thesis
{

CeresIK::CeresIK(ros::NodeHandle &nh, ros::NodeHandle &priv_nh): nh(nh), priv_nh(priv_nh), rand_gen(rand_dev())
{
  // init robot monitor
  robot_monitor = std::make_shared<RobotMonitor>(nh, priv_nh);

  // init planning_scene_monitor
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  planning_scene_monitor->setStateUpdateFrequency(100);
  //planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startStateMonitor();

  // init visual_tools
  visual_tools.reset(new moveit_visual_tools::MoveItVisualTools(planning_scene_monitor->getRobotModel()->getModelFrame(), "/ceres_ik_visual_markers"));

  // TODO
  endpoint_sub = priv_nh.subscribe("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback", 1, &CeresIK::subscriberCallback, this);

  joint_controller = std::make_shared<JointTrajectoryControlInterface>(nh);
  joint_controller->start("");

  loop(); // TODO
}

void CeresIK::subscriberCallback(const visualization_msgs::InteractiveMarkerFeedbackPtr &msg)
{
  auto given_endpoint = msg->pose;
  ROS_WARN_THROTTLE(1.0, "Endpoint received. x: %lf, y: %lf, z: %lf", given_endpoint.position.x, given_endpoint.position.y, given_endpoint.position.z);
  endpoint.x() =  given_endpoint.position.x;
  endpoint.y() =  given_endpoint.position.y;
  endpoint.z() =  given_endpoint.position.z;
  direction = Eigen::Quaterniond(given_endpoint.orientation.w, given_endpoint.orientation.x, given_endpoint.orientation.y, given_endpoint.orientation.z);
  endpoint_received = true;
}

moveit::core::RobotState CeresIK::getCurrentRobotState()
{
  // TODO: this method is slow... 
  planning_scene_monitor->waitForCurrentRobotState(ros::Time(0));
  planning_scene_monitor::LockedPlanningSceneRO lps(planning_scene_monitor);
  return lps->getCurrentState(); // copy the current state
}

// TODO: this is just a dummy loop function. this will run on a separate thread!
void CeresIK::loop()
{
  moveit::core::RobotState robot_state = getCurrentRobotState();

  ros::Rate r(20);
  while (ros::ok())
  {
    ros::spinOnce();
    moveit::core::RobotState robot_state = getCurrentRobotState();

    if (!endpoint_received)
    {
      r.sleep();
      continue;
    }

    if (!update(robot_state))
    {
      ROS_ERROR("Can't find a solution!");
      r.sleep();
      continue;
    }

    // TODO: control robot
    auto controller_state = joint_controller->getState();
    if (controller_state!=nullptr)
    {
      //joint_controller->setJointPositions(robot_state.getVariablePositions());
    }

    visual_tools->publishRobotState(robot_state);
    r.sleep();

    // TODO: control robot
    //if (controller_state!=nullptr)
    //  robot_state.setVariablePositions(controller_state->actual.positions.data());
  }
}

bool CeresIK::update(moveit::core::RobotState &current_state)
{
  ROS_INFO_ONCE("Number of variables in the robot state: %d", (int)(current_state.getVariableCount()) );
  if (robot::num_variables != current_state.getVariableCount())
  {
    ROS_FATAL_ONCE("Number of variables does not match!");
    exit(-1);
  }

  double* variable_positions = current_state.getVariablePositions();

  // Generate target_positions (this is the init state and this will be optimized)
  double target_positions[robot::num_targets];
  for (int i=0; i<robot::num_targets; i++)
  {
    const int joint_i = robot::target_idx_to_joint_idx[i];
    const int variable_i = robot::joint_idx_to_variable_idx[joint_i];
    target_positions[i] = variable_positions[variable_i];

    // Add noise to the init state to avoid gimball lock, etc.
    double noise = 0.0; // TODO: 0.1
    if (noise>0)
    {
      double sampling_min = target_positions[i]-noise;
      if (sampling_min<robot::joint_min_position[joint_i]) sampling_min=robot::joint_min_position[joint_i];
      double sampling_max = target_positions[i]+noise;
      if (sampling_max>robot::joint_max_position[joint_i]) sampling_max=robot::joint_max_position[joint_i];
      std::uniform_real_distribution<> dis(sampling_min, sampling_max);
      target_positions[i] = dis(rand_gen);
    }
  }

  // Generate const_target_positions (this a noise-free copy of target_positions and should not be modified!)
  double const_target_positions[robot::num_targets];
  for (int i=0; i<robot::num_targets; i++)
  {
    const int joint_i = robot::target_idx_to_joint_idx[i];
    const int variable_i = robot::joint_idx_to_variable_idx[joint_i];
    const_target_positions[i] = variable_positions[variable_i];
  }

  // Generate target centers. Assuming that center=(max+min)/2
  /*
  double target_centers[robot::num_targets];
  for (int i=0; i<robot::num_targets; i++)
  {
    const int joint_i = robot::target_idx_to_joint_idx[i];
    target_centers[i] = 0.5 * (robot::joint_max_position[joint_i] + robot::joint_min_position[joint_i]);
  }
  */

  // TODO: Find the partial kinematic tree for the endpoint goal
  // int current_joint_idx = robot::endpoint_joint_idx;
  // .....

  JointLinkCollisionStateConstPtr state = robot_monitor->getState();
  const std::vector<CollisionObject*> int_objects = robot_monitor->getInternalObjects();

  ceres::Problem problem;

  // ========== Preferred Joint Position Goal ==========
  ceres::CostFunction* preferred_joint_position_goal = PreferredJointPositionGoal::Create();
  ceres::TukeyLoss *preferred_joint_position_loss = new ceres::TukeyLoss(0.1); // goal weight
  problem.AddResidualBlock(preferred_joint_position_goal, preferred_joint_position_loss, target_positions);
  //problem.AddResidualBlock(preferred_joint_position_goal, robot::createPreferredJointPositionLoss(), target_positions); // TODO

  // ========== Minimal Joint Displacement Goal ==========
  //ceres::CostFunction* minimal_joint_displacement_goal = MinimalJointDisplacementGoal::Create(const_target_positions);
  //ceres::CauchyLoss *minimal_joint_displacement_loss = new ceres::CauchyLoss(0.1); // goal weight
  //problem.AddResidualBlock(minimal_joint_displacement_goal, minimal_joint_displacement_loss, target_positions);

  // ================== Endpoint Goal ==================
  ceres::CostFunction* endpoint_goal = EndpointGoal::Create(endpoint, direction, variable_positions);
  //ceres::HuberLoss *endpoint_loss = new ceres::HuberLoss(1.0); // goal weight
  ceres::RelaxedIKLoss *endpoint_loss = new ceres::RelaxedIKLoss(0.5); // goal weight
  problem.AddResidualBlock(endpoint_goal, endpoint_loss, target_positions);
  //problem.AddResidualBlock(endpoint_goal, robot::createEndpointLoss(), target_positions);

  // ============= Collision Avoidance Goal ============
  ceres::CostFunction* collision_avoidance_goal = CollisionAvoidanceGoal::Create(variable_positions,
                                                                                 state->collision_state.int_pair_a.data(),
                                                                                 state->collision_state.int_pair_b.data(),
                                                                                 state->collision_state.int_pair_a.size(),
                                                                                 int_objects);
  //ceres::HuberLoss *collision_avoidance_loss = new ceres::HuberLoss(1.0); // goal weight
  problem.AddResidualBlock(collision_avoidance_goal, nullptr, target_positions);
  //problem.AddResidualBlock(collision_avoidance_goal, robot::createCollisionAvoidanceLoss(), target_positions);





  // Target min/max constraints
  for (int i=0; i<robot::num_targets; i++)
  {
    const int joint_i = robot::target_idx_to_joint_idx[i];
    if (!robot::joint_is_position_bounded[joint_i])
    {
      ROS_WARN("Target is not bounded. This can cause bad solutions!");
      continue;
    }

    // TODO: ALTERNATIVE WAY?
    double min_val = const_target_positions[i] - 0.1;
    double max_val = const_target_positions[i] + 0.1;
    if (robot::joint_min_position[joint_i] > min_val) min_val = robot::joint_min_position[joint_i];
    if (robot::joint_max_position[joint_i] < max_val) max_val = robot::joint_max_position[joint_i];
    problem.SetParameterLowerBound(target_positions, i, min_val); 
    problem.SetParameterUpperBound(target_positions, i, max_val); 

    // STANDARD WAY OF SETTING JOINT LIMITS
    //problem.SetParameterLowerBound(target_positions, i, robot::joint_min_position[joint_i]); 
    //problem.SetParameterUpperBound(target_positions, i, robot::joint_max_position[joint_i]); 
  }

  ceres::Solver::Options options;
  robot::setSolverOptions(options);
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

  if (!summary.IsSolutionUsable())
  {
    return false;
  }

  // Update robot state
  for (int i=0; i<robot::num_targets; i++)
  {
    const int joint_idx = robot::target_idx_to_joint_idx[i];
    const int variable_idx = robot::joint_idx_to_variable_idx[joint_idx];
    variable_positions[variable_idx] = target_positions[i];

    // int state_idx = current_state.getVariablePosition(robot::joint_names[joint_idx]) !!!!!!!!!!!
    printf("%f ", target_positions[i]);
  }
  printf("\n");
  current_state.update(true); // TODO: can be faster with: updateLinkTransforms()

  // TODOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
  auto controller_state = joint_controller->getState();
  if (controller_state!=nullptr)
  {
    joint_controller->setJointPositions(target_positions);
  }

  return true;
}


} // namespace salih_marangoz_thesis