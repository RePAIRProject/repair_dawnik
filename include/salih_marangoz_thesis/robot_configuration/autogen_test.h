#ifndef __AUTOGENERATED_ROBOT_CONFIGURATION__
#define __AUTOGENERATED_ROBOT_CONFIGURATION__

#include <vector>
#include <string>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <hpp/fcl/shape/geometric_shapes.h>

using namespace ceres;

using hpp::fcl::CollisionObject;
// supported collision shapes:
using hpp::fcl::Box;
using hpp::fcl::Sphere;
using hpp::fcl::Capsule;
using hpp::fcl::Cylinder;
using hpp::fcl::Plane;

namespace robot
{

// Util
template<typename T>
CollisionObject* inflatedCollisionObject(const T &shape, double inflation)
{
  return new CollisionObject(std::make_shared<T>(shape.inflated(inflation).first));
}

// Constants
const int endpoint_link_idx = 8;
const int num_joints = 9;
const int num_variables = 6;
const int num_links = 9;
const int num_objects = 19;
const int num_acm_link_pairs = 18;
const int num_targets = 6;

// Mapping vectors
const int joint_idx_to_variable_idx[9] = {-1,-1,0,1,2,3,4,5,-1}; // -1 if no variable available. Can be used as joint_has_variable vector
const int variable_idx_to_joint_idx[6] = {2,3,4,5,6,7};
const int joint_idx_to_target_idx[9] = {-1,-1,0,1,2,3,4,5,-1};
const int target_idx_to_joint_idx[6] = {2,3,4,5,6,7};
const int object_idx_to_link_idx[19] = {1,2,2,3,3,3,3,3,3,4,4,4,5,5,5,5,5,6,7};

// Joint info
const std::string joint_names[9] = {"ASSUMED_FIXED_ROOT_JOINT","world_joint","joint1","joint2","joint3","joint4","joint5","joint6","joint_eef"};
const int joint_child_link_idx[9] = {0,1,2,3,4,5,6,7,8};
const int joint_parent_link_idx[9] = {-1,0,1,2,3,4,5,6,7}; // -1 if no link available
const int joint_is_position_bounded[9] = {0,0,1,1,1,1,1,1,0}; // bool
const double joint_preferred_position[9] = {0.000000,0.000000,0.000000,0.000000,2.587451,0.000000,0.000000,0.000000,0.000000};
const double joint_max_position[9] = {0.000000,0.000000,6.283185,2.617990,5.235988,6.283185,2.164200,6.283185,0.000000};
const double joint_min_position[9] = {0.000000,0.000000,-6.283185,-2.617990,-0.061087,-6.283185,-2.164200,-6.283185,0.000000};
const int joint_is_velocity_bounded[9] = {0,0,1,1,1,1,1,1,0}; // bool
const double joint_max_velocity[9] = {0.000000,0.000000,2.140000,2.140000,2.140000,2.140000,2.140000,2.140000,0.000000};
const double joint_min_velocity[9] = {0.000000,0.000000,-2.140000,-2.140000,-2.140000,-2.140000,-2.140000,-2.140000,0.000000};
const int joint_is_acceleration_bounded[9] = {0,0,0,0,0,0,0,0,0}; // bool
const double joint_max_acceleration[9] = {0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000};
const double joint_min_acceleration[9] = {0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000};

// Link info
const std::string link_names[9] = {"world","link_base","link1","link2","link3","link4","link5","link6","link_eef"};
const int link_parent_joint_idx[9] = {0,1,2,3,4,5,6,7,8};
const double link_transform_translation_only[9][3] = {{0.000000, 0.000000, 0.000000},
                                                      {0.000000, 0.000000, 0.000000},
                                                      {0.000000, 0.000000, 0.243300},
                                                      {0.000000, 0.000000, 0.000000},
                                                      {0.200000, 0.000000, 0.000000},
                                                      {0.087000, -0.227600, 0.000000},
                                                      {0.000000, 0.000000, 0.000000},
                                                      {0.000000, 0.061500, 0.000000},
                                                      {0.000000, 0.000000, 0.000000}};
const double link_transform_quaternion_only[9][4] = {{1.000000, 0.000000, 0.000000, 0.000000},
                                                     {1.000000, 0.000000, 0.000000, 0.000000},
                                                     {1.000000, 0.000000, 0.000000, 0.000000},
                                                     {0.500004, -0.499998, -0.500002, -0.499996},
                                                     {0.000003, 0.707105, 0.707108, 0.000003},
                                                     {0.707105, 0.707108, 0.000000, 0.000000},
                                                     {0.707105, 0.707108, 0.000000, 0.000000},
                                                     {0.707105, -0.707108, 0.000000, 0.000000},
                                                     {1.000000, 0.000000, 0.000000, 0.000000}};
const int link_can_skip_translation[9] = {1,1,0,1,0,0,1,0,1}; // bool
const int link_can_skip_rotation[9] = {1,1,1,0,0,0,0,0,1}; // bool

// ACM
const int acm[9][9]= {{1,1,1,1,1,1,1,1,1},
                      {1,1,1,1,0,0,0,0,1},
                      {1,1,1,1,1,0,0,0,1},
                      {1,1,1,1,1,1,0,0,1},
                      {1,0,1,1,1,1,1,1,1},
                      {1,0,0,1,1,1,1,1,1},
                      {1,0,0,0,1,1,1,1,1},
                      {1,0,0,0,1,1,1,1,1},
                      {1,1,1,1,1,1,1,1,1}};

// Objective weights
const double weight_preferred_joint_position_goal[9] = {1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000,1.000000};

// Collision objects info
const double object_transform_translation_only[19][3] = {{0.000000, 0.000000, 0.040000},
                                                         {0.000000, 0.000000, 0.000000},
                                                         {0.000000, 0.000000, -0.075000},
                                                         {0.000000, 0.000000, 0.050000},
                                                         {0.000000, 0.000000, 0.100000},
                                                         {0.065000, 0.000000, 0.100000},
                                                         {0.130000, 0.000000, 0.100000},
                                                         {0.195000, 0.000000, 0.100000},
                                                         {0.195000, 0.000000, 0.050000},
                                                         {0.000000, 0.000000, 0.000000},
                                                         {0.045000, 0.000000, 0.000000},
                                                         {0.090000, 0.000000, 0.000000},
                                                         {0.000000, 0.000000, -0.160000},
                                                         {0.000000, 0.000000, -0.100000},
                                                         {0.000000, -0.050000, -0.100000},
                                                         {0.000000, -0.050000, -0.050000},
                                                         {0.000000, -0.050000, 0.000000},
                                                         {0.000000, 0.000000, 0.000000},
                                                         {0.000000, 0.000000, 0.000000}};
const double object_transform_quaternion_only[19][4] = {{1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000},
                                                        {1.000000, 0.000000, 0.000000, 0.000000}};
const int object_can_skip_translation[19] = {0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,1}; // bool
const int object_can_skip_rotation[19] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}; // bool

// Collision Objects Function
const double inflation = 0.05;
static inline std::vector<CollisionObject*> getRobotCollisionObjects()
{
  std::vector<CollisionObject*> objects;

  objects.reserve(19);
  objects.push_back( inflatedCollisionObject(Sphere(0.1), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );
  objects.push_back( inflatedCollisionObject(Sphere(0.05), inflation) );

  return objects;
}

// Collision Constraint Function
inline static void setProblemConstraints(ceres::Problem &problem, double *targets_ptr, double *targets_init)
{
}

// Solver Options
static inline void setSolverOptions(ceres::Solver::Options &options)
{
  options.eta = DBL_MIN;
  options.function_tolerance = DBL_MIN;
  options.gradient_tolerance = DBL_MIN;
  options.jacobi_scaling = true;
  options.linear_solver_type = DENSE_QR;
  options.max_num_iterations = 999;
  options.max_solver_time_in_seconds = 0.01;
  options.minimizer_progress_to_stdout = true;
  options.minimizer_type = TRUST_REGION;
  options.parameter_tolerance = DBL_MIN;
}

} // namespace robot

#endif // __AUTOGENERATED_ROBOT_CONFIGURATION__
