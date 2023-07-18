#! /usr/bin/env python

import rospy
import tf
import numpy as np

from dawn_ik.msg import IKGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose, Twist
import sys
from scipy import interpolate
from visualization_msgs.msg import Marker
import time, os
import yaml
import json

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))

AXES = "szyx"

traj_pub = None
listener = None
current_joint_state = None

def publish_ee_goal(x, y, z, roll, pitch, yaw):
  dawnik_goal = IKGoal()
  dawnik_goal.mode = IKGoal.MODE_1 + IKGoal.MODE_2
  dawnik_goal.m1_x = x
  dawnik_goal.m1_y = y
  dawnik_goal.m1_z = z
  dawnik_goal.m1_limit_dist = 0.1
  dawnik_goal.m1_weight = 4
  quad = tf.transformations.quaternion_from_euler(roll, pitch, yaw, axes=AXES)
  dawnik_goal.m2_x = quad[0]
  dawnik_goal.m2_y = quad[1]
  dawnik_goal.m2_z = quad[2]
  dawnik_goal.m2_w = quad[3]
  dawnik_goal.m2_weight = 2
  dawn_ik_goal_pub.publish(dawnik_goal)

  marker = Marker()
  pose = Pose()
  pose.position.x = x
  pose.position.y = y
  pose.position.z = z
  #quad = tf.transformations.quaternion_from_euler(roll, pitch, yaw, axes=AXES)
  pose.orientation.x =  quad[0]
  pose.orientation.y =  quad[1]
  pose.orientation.z =  quad[2]
  pose.orientation.w =  quad[3]
  marker.header.frame_id = "world"
  marker.type = marker.SPHERE
  marker.id = 53
  marker.action = marker.ADD
  marker.pose = pose
  marker.lifetime = rospy.Duration()
  marker.scale.x = 0.05
  marker.scale.y = 0.05
  marker.scale.z = 0.05
  marker.color.a = 1.0
  marker.color.r = 1.0
  marker_pub.publish(marker)

def track_joint_state(msg):
  global current_joint_state
  current_joint_state = msg

if __name__ == "__main__":
  rospy.init_node('run_experiment')
  waypoints_file = rospy.get_param("~waypoints_file", SCRIPT_DIR + "/waypoints.txt")
  publish_rate = rospy.get_param("~publish_rate", 100.0)
  world_frame = rospy.get_param("~world_frame", "world")
  endpoint_frame = rospy.get_param("~endpoint_frame", "head_link_eef")

  listener = tf.TransformListener()
  dawn_ik_goal_pub = rospy.Publisher("/dawn_ik_solver/ik_goal", IKGoal, queue_size=5)
  marker_pub = rospy.Publisher("~goal_marker", Marker, queue_size = 5)
  rospy.Subscriber("/joint_states", JointState, track_joint_state)

  print("Reading file:", waypoints_file)
  waypoints = np.loadtxt(waypoints_file)
  w_t = waypoints[:,0]
  w_x = waypoints[:,1]
  w_y = waypoints[:,2]
  w_z = waypoints[:,3]
  w_roll = waypoints[:,4]
  w_pitch = waypoints[:,5]
  w_yaw = waypoints[:,6]
  t_max = np.max(w_t)

  f_x = interpolate.interp1d(w_t, w_x)
  f_y = interpolate.interp1d(w_t, w_y)
  f_z = interpolate.interp1d(w_t, w_z)
  f_roll = interpolate.interp1d(w_t, w_roll)
  f_pitch = interpolate.interp1d(w_t, w_pitch)
  f_yaw = interpolate.interp1d(w_t, w_yaw)

  rate = rospy.Rate(publish_rate)
  t = 0.0
  rospy.sleep(1)
  entries = []
  while not rospy.is_shutdown():
    # Experiment entries...
    entry = {}

    # ENTRY CURRENT TIME
    entry["time"] = t

    # ENTRY: EE GOAL
    x = f_x(t)
    y = f_y(t)
    z = f_z(t)
    roll = f_roll(t)
    pitch = f_pitch(t)
    yaw = f_yaw(t)
    publish_ee_goal(x,y,z,roll,pitch,yaw)
    entry["ee_goal"] = {"x": float(x), "y": float(y), "z": float(z), "roll": float(roll), "pitch": float(pitch), "yaw": float(yaw)}

    #################### wait for robots to move #################################
    rate.sleep()

    # ENTRY: EE CURRENT
    try:
      (curr_trans, curr_rot) = listener.lookupTransform(world_frame, endpoint_frame, rospy.Time(0))
      (curr_roll, curr_pitch, curr_yaw) = tf.transformations.euler_from_quaternion(curr_rot)
      curr_x = curr_trans[0]
      curr_y = curr_trans[1]
      curr_z = curr_trans[2]
    except Exception as e:
      rospy.logerr("tf error. experiment failed!")
      print(e)
      exit(-1)
    entry["ee_curr"] = {"x": float(curr_x), "y": float(curr_y), "z": float(curr_z), "roll": float(curr_roll), "pitch": float(curr_pitch), "yaw": float(curr_yaw)}

    # ENTRY: JOINT STATES
    entry["joint_names"] = list(current_joint_state.name)
    entry["joint_positions"] = list(current_joint_state.position)
    entry["joint_velocities"] = list(current_joint_state.velocity)
    entry["joint_efforts"] = list(current_joint_state.effort)


    # TODO: DEBUG ROTATIONS.............
    print(entry["ee_goal"])
    print(entry["ee_curr"])

    entries.append(entry)

    t += 1.0/publish_rate
    if t>t_max:break

  #rospy.spin()

  # save results to a file
  timestr = time.strftime("%Y%m%d-%H%M%S")
  
  out_filename_default = SCRIPT_DIR + "/../../results/experiment_" + timestr + ".json" # or yaml
  out_filename = rospy.get_param("~out_filename", out_filename_default)
  if out_filename == "":
    out_filename = out_filename_default

  with open(out_filename, "w") as f:
    #f.write(yaml.dump(entries, default_flow_style=None, sort_keys=False))
    f.write(json.dumps(entries))

  print("Experiment Finished!")
