from __future__ import print_function
from __future__ import division

try:
    # ROS2 imports
    from tf_transformations import quaternion_multiply, quaternion_from_euler  # Use tf_transformations package
    from geometry_msgs.msg import Quaternion
    import rclpy
    from rclpy.node import Node
    # For ROS2, we'll need to adapt MoveIt integration
    # from moveit_py import PlanningComponent, PlanningInterface  # Will need proper MoveIt2 Python bindings
    ROS2_AVAILABLE = True
except ImportError:
    # ROS1 fallback
    from tf.transformations import quaternion_multiply, quaternion_from_euler
    from geometry_msgs.msg import Quaternion
    from moveit_commander import MoveGroupCommander
    import rospy
    ROS2_AVAILABLE = False

import numpy as np
from itertools import chain
try:
    from itertools import izip as zip
except ImportError: # will be 3.x series
    pass
from copy import deepcopy
import math


class CalibrationMovements:
    def __init__(self, move_group_name, max_velocity_scaling, max_acceleration_scaling, angle_delta, translation_delta, move_group_namespace='/', node=None):
        self.node = node
        
        if ROS2_AVAILABLE and node is not None:  # ROS2
            # For now, we'll implement a placeholder for MoveIt2 integration
            # This will need to be properly implemented with MoveIt2 Python bindings when they're available
            self.node.get_logger().warn("MoveIt2 Python integration not yet fully implemented in this port. Robot movements will need manual implementation.")
            self.mgc = None  # Placeholder
            self.start_pose = None  # Will need to be set appropriately
            
            # Store parameters for later use
            self.move_group_name = move_group_name
            self.move_group_namespace = move_group_namespace
            self.max_velocity_scaling = max_velocity_scaling
            self.max_acceleration_scaling = max_acceleration_scaling
            
        else:  # ROS1
            # self.client = HandeyeClient()  # TODO: move around marker when eye_on_hand, automatically take samples via trigger topic
            if not move_group_namespace.endswith('/'):
                move_group_namespace = move_group_namespace + '/'
            if move_group_namespace != '/':
                self.mgc = MoveGroupCommander(move_group_name, robot_description=move_group_namespace+'robot_description', ns=move_group_namespace)
            else:
                self.mgc = MoveGroupCommander(move_group_name)
            self.mgc.set_planner_id("RRTConnectkConfigDefault")  # TODO: this is only needed for the UR5
            self.mgc.set_max_velocity_scaling_factor(max_velocity_scaling)
            self.mgc.set_max_acceleration_scaling_factor(max_acceleration_scaling)
            self.start_pose = self.mgc.get_current_pose()
            
        self.joint_limits = [math.radians(90)] * 5 + [math.radians(180)] + [math.radians(350)]  # TODO: make param
        self.target_poses = []
        self.current_pose_index = None
        self.current_plan = None
        self.fallback_joint_limits = [math.radians(90)] * 4 + [math.radians(90)] + [math.radians(180)] + [
            math.radians(350)]
            
        if not ROS2_AVAILABLE and self.mgc is not None:
            if len(self.mgc.get_active_joints()) == 6:
                self.fallback_joint_limits = self.fallback_joint_limits[1:]
                
        self.angle_delta = angle_delta
        self.translation_delta = translation_delta
        
    def _log_info(self, message):
        """Helper method for logging that works with both ROS1 and ROS2"""
        if self.node is not None:  # ROS2
            self.node.get_logger().info(message)
        else:  # ROS1
            import rospy
            rospy.loginfo(message)
            
    def _log_error(self, message):
        """Helper method for error logging that works with both ROS1 and ROS2"""
        if self.node is not None:  # ROS2
            self.node.get_logger().error(message)
        else:  # ROS1
            import rospy
            rospy.logerr(message)
            
    def _log_warn(self, message):
        """Helper method for warning logging that works with both ROS1 and ROS2"""
        if self.node is not None:  # ROS2
            self.node.get_logger().warn(message)
        else:  # ROS1
            import rospy
            rospy.logwarn(message)

    def set_and_check_starting_position(self):
        # sets the starting position to the current robot cartesian EE position and checks movement in all directions
        # TODO: make repeatable
        #  - save the home position and each target position as joint position
        #  - plan to each target position and back, save plan if not crazy
        #  - return true if can plan to each target position without going crazy
        self.start_pose = self.mgc.get_current_pose()
        self.target_poses = self._compute_poses_around_state(self.start_pose, self.angle_delta, self.translation_delta)
        self.current_pose_index = -1
        ret = self._check_target_poses(self.joint_limits)
        if ret:
            self._log_info("Set current pose as home")
            return True
        else:
            self._log_error("Can't calibrate from this position!")
            self.start_pose = None
            self.target_poses = None
            return False

    def select_target_pose(self, i):
        number_of_target_poses = len(self.target_poses)
        if 0 <= i < number_of_target_poses:
            self._log_info("Selected pose {} for next movement".format(i))
            self.current_pose_index = i
            return True
        else:
            self._log_error("Index {} is out of bounds: there are {} target poses".format(i, number_of_target_poses))
            return False

    def plan_to_start_pose(self):
        # TODO: use joint position http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html#planning-to-a-joint-goal
        self._log_info("Planning to home pose")
        return self._plan_to_pose(self.start_pose)

    def plan_to_current_target_pose(self):
        # TODO: use joint position http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html#planning-to-a-joint-goal
        i = self.current_pose_index
        self._log_info("Planning to target pose {}".format(i))
        return self._plan_to_pose(self.target_poses[i])

    def execute_plan(self):
        if self.plan is None:
            self._log_error("No plan found!")
            return False
        if CalibrationMovements._is_crazy_plan(self.plan, self.fallback_joint_limits):
            self._log_error("Crazy plan found, not executing!")
            return False
        self.mgc.execute(self.plan)
        return True

    def _plan_to_pose(self, pose):
        self.mgc.set_start_state_to_current_state()
        self.mgc.set_pose_target(pose)
        ret = self.mgc.plan()
        if type(ret) is tuple:
            # noetic
            success, plan, planning_time, error_code = ret
        else:
            # melodic
            plan = ret
        if CalibrationMovements._is_crazy_plan(plan, self.fallback_joint_limits):
            self._log_warn("Planning failed")
            self.plan = None
            return False
        else:
            self._log_info("Planning successful")
            self.plan = plan
            return True

    def _check_target_poses(self, joint_limits):
        if len(self.fallback_joint_limits) == 6:
            joint_limits = joint_limits[1:]
        for fp in self.target_poses:
            self.mgc.set_pose_target(fp)
            ret = self.mgc.plan()
            if type(ret) is tuple:
                # noetic
                success, plan, planning_time, error_code = ret
            else:
                # melodic
                plan = ret
            if len(plan.joint_trajectory.points) == 0 or CalibrationMovements._is_crazy_plan(plan, joint_limits):
                return False
        return True

    @staticmethod
    def _compute_poses_around_state(start_pose, angle_delta, translation_delta):
        basis = np.eye(3)

        pos_deltas = [quaternion_from_euler(*rot_axis * angle_delta) for rot_axis in basis]
        neg_deltas = [quaternion_from_euler(*rot_axis * (-angle_delta)) for rot_axis in basis]

        quaternion_deltas = list(chain.from_iterable(zip(pos_deltas, neg_deltas)))  # interleave

        final_rots = []
        for qd in quaternion_deltas:
            final_rots.append(list(qd))

        # TODO: accept a list of delta values

        pos_deltas = [quaternion_from_euler(*rot_axis * angle_delta / 2) for rot_axis in basis]
        neg_deltas = [quaternion_from_euler(*rot_axis * (-angle_delta / 2)) for rot_axis in basis]

        quaternion_deltas = list(chain.from_iterable(zip(pos_deltas, neg_deltas)))  # interleave
        for qd in quaternion_deltas:
            final_rots.append(list(qd))

        final_poses = []
        for rot in final_rots:
            fp = deepcopy(start_pose)
            ori = fp.pose.orientation
            combined_rot = quaternion_multiply([ori.x, ori.y, ori.z, ori.w], rot)
            fp.pose.orientation = Quaternion(*combined_rot)
            final_poses.append(fp)

        fp = deepcopy(start_pose)
        fp.pose.position.x += translation_delta / 2
        final_poses.append(fp)

        fp = deepcopy(start_pose)
        fp.pose.position.x -= translation_delta / 2
        final_poses.append(fp)

        fp = deepcopy(start_pose)
        fp.pose.position.y += translation_delta
        final_poses.append(fp)

        fp = deepcopy(start_pose)
        fp.pose.position.y -= translation_delta
        final_poses.append(fp)

        fp = deepcopy(start_pose)
        fp.pose.position.z += translation_delta / 3
        final_poses.append(fp)

        return final_poses

    @staticmethod
    def _rot_per_joint(plan, degrees=False):
        np_traj = np.array([p.positions for p in plan.joint_trajectory.points])
        if len(np_traj) == 0:
            raise ValueError
        np_traj_max_per_joint = np_traj.max(axis=0)
        np_traj_min_per_joint = np_traj.min(axis=0)
        ret = abs(np_traj_max_per_joint - np_traj_min_per_joint)
        if degrees:
            ret = [math.degrees(j) for j in ret]
        return ret

    @staticmethod
    def _is_crazy_plan(plan, max_rotation_per_joint):
        abs_rot_per_joint = CalibrationMovements._rot_per_joint(plan)
        if (abs_rot_per_joint > max_rotation_per_joint).any():
            return True
        else:
            return False
