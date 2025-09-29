try:
    # ROS2 imports
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Empty
    ROS2_AVAILABLE = True
except ImportError:
    # ROS1 imports
    import rospy
    import std_srvs
    from std_srvs import srv
    ROS2_AVAILABLE = False

import easy_handeye as hec
from easy_handeye.handeye_calibration import HandeyeCalibrationParameters
import easy_handeye_msgs as ehm
from easy_handeye_msgs import srv


class HandeyeClient(Node if ROS2_AVAILABLE else object):

    def __init__(self, namespace=None, node_name='handeye_client'):
        if ROS2_AVAILABLE:  # ROS2
            super().__init__(node_name)
            
            if namespace is None:
                namespace = self.get_namespace()

            self.parameters = None
            self.list_algorithms_proxy = None
            self.set_algorithm_proxy = None
            self.get_sample_proxy = None
            self.take_sample_proxy = None
            self.remove_sample_proxy = None
            self.compute_calibration_proxy = None
            self.save_calibration_proxy = None
            self.check_starting_pose_proxy = None
            self.enumerate_target_poses_proxy = None
            self.select_target_pose_proxy = None
            self.plan_to_selected_target_pose_proxy = None
            self.execute_plan_proxy = None

            if namespace != "/":
                self.get_logger().info(f"Configuring for calibration with namespace: {namespace}")
                self.set_namespace(namespace)
            else:
                self.get_logger().warn("No namespace was passed at construction; remember to use set_namespace()")
                
        else:  # ROS1
            if namespace is None:
                namespace = rospy.get_namespace()

            self.parameters = None
            self.list_algorithms_proxy = None
            self.set_algorithm_proxy = None
            self.get_sample_proxy = None
            self.take_sample_proxy = None
            self.remove_sample_proxy = None
            self.compute_calibration_proxy = None
            self.save_calibration_proxy = None
            self.check_starting_pose_proxy = None
            self.enumerate_target_poses_proxy = None
            self.select_target_pose_proxy = None
            self.plan_to_selected_target_pose_proxy = None
            self.execute_plan_proxy = None

            if namespace != "/":
                rospy.loginfo("Configuring for calibration with namespace: {}".format(namespace))
                self.set_namespace(namespace)
            else:
                rospy.logwarn("No namespace was passed at construction; remember to use set_namespace()")

    def set_namespace(self, namespace):
        if ROS2_AVAILABLE:  # ROS2
            if hasattr(self, 'get_logger'):
                node = self
            else:
                node = None
            self.parameters = HandeyeCalibrationParameters.init_from_parameter_server(namespace, node)

            # init services: sampling
            self.get_sample_proxy = self.create_client(ehm.srv.TakeSample, hec.GET_SAMPLE_LIST_TOPIC)
            self.take_sample_proxy = self.create_client(ehm.srv.TakeSample, hec.TAKE_SAMPLE_TOPIC)
            self.remove_sample_proxy = self.create_client(ehm.srv.RemoveSample, hec.REMOVE_SAMPLE_TOPIC)

            # init services: calibration
            self.list_algorithms_proxy = self.create_client(ehm.srv.ListAlgorithms, hec.LIST_ALGORITHMS_TOPIC)
            self.set_algorithm_proxy = self.create_client(ehm.srv.SetAlgorithm, hec.SET_ALGORITHM_TOPIC)
            self.compute_calibration_proxy = self.create_client(ehm.srv.ComputeCalibration, hec.COMPUTE_CALIBRATION_TOPIC)
            self.save_calibration_proxy = self.create_client(Empty, hec.SAVE_CALIBRATION_TOPIC)

            if not self.parameters.freehand_robot_movement:
                # init services: robot movement
                self.check_starting_pose_proxy = self.create_client(ehm.srv.CheckStartingPose, hec.CHECK_STARTING_POSE_TOPIC)
                self.enumerate_target_poses_proxy = self.create_client(ehm.srv.EnumerateTargetPoses, hec.ENUMERATE_TARGET_POSES_TOPIC)
                self.select_target_pose_proxy = self.create_client(ehm.srv.SelectTargetPose, hec.SELECT_TARGET_POSE_TOPIC)
                self.plan_to_selected_target_pose_proxy = self.create_client(ehm.srv.PlanToSelectedTargetPose, hec.PLAN_TO_SELECTED_TARGET_POSE_TOPIC)
                self.execute_plan_proxy = self.create_client(ehm.srv.ExecutePlan, hec.EXECUTE_PLAN_TOPIC)
                
        else:  # ROS1
            self.parameters = HandeyeCalibrationParameters.init_from_parameter_server(namespace)

            # init services: sampling
            rospy.wait_for_service(hec.GET_SAMPLE_LIST_TOPIC)
            self.get_sample_proxy = rospy.ServiceProxy(hec.GET_SAMPLE_LIST_TOPIC, ehm.srv.TakeSample)
            rospy.wait_for_service(hec.TAKE_SAMPLE_TOPIC)
            self.take_sample_proxy = rospy.ServiceProxy(hec.TAKE_SAMPLE_TOPIC, ehm.srv.TakeSample)
            rospy.wait_for_service(hec.REMOVE_SAMPLE_TOPIC)
            self.remove_sample_proxy = rospy.ServiceProxy(hec.REMOVE_SAMPLE_TOPIC, ehm.srv.RemoveSample)

            # init services: calibration
            rospy.wait_for_service(hec.LIST_ALGORITHMS_TOPIC)
            self.list_algorithms_proxy = rospy.ServiceProxy(hec.LIST_ALGORITHMS_TOPIC, ehm.srv.ListAlgorithms)
            rospy.wait_for_service(hec.SET_ALGORITHM_TOPIC)
            self.set_algorithm_proxy = rospy.ServiceProxy(hec.SET_ALGORITHM_TOPIC, ehm.srv.SetAlgorithm)
            rospy.wait_for_service(hec.COMPUTE_CALIBRATION_TOPIC)
            self.compute_calibration_proxy = rospy.ServiceProxy(hec.COMPUTE_CALIBRATION_TOPIC, ehm.srv.ComputeCalibration)
            rospy.wait_for_service(hec.SAVE_CALIBRATION_TOPIC)
            self.save_calibration_proxy = rospy.ServiceProxy(hec.SAVE_CALIBRATION_TOPIC, std_srvs.srv.Empty)

            if not self.parameters.freehand_robot_movement:
                # init services: robot movement
                rospy.wait_for_service(hec.CHECK_STARTING_POSE_TOPIC)
                self.check_starting_pose_proxy = rospy.ServiceProxy(hec.CHECK_STARTING_POSE_TOPIC, ehm.srv.CheckStartingPose)
                rospy.wait_for_service(hec.ENUMERATE_TARGET_POSES_TOPIC)
                self.enumerate_target_poses_proxy = rospy.ServiceProxy(hec.ENUMERATE_TARGET_POSES_TOPIC, ehm.srv.EnumerateTargetPoses)
                rospy.wait_for_service(hec.SELECT_TARGET_POSE_TOPIC)
                self.select_target_pose_proxy = rospy.ServiceProxy(hec.SELECT_TARGET_POSE_TOPIC, ehm.srv.SelectTargetPose)
                rospy.wait_for_service(hec.PLAN_TO_SELECTED_TARGET_POSE_TOPIC)
                self.plan_to_selected_target_pose_proxy = rospy.ServiceProxy(hec.PLAN_TO_SELECTED_TARGET_POSE_TOPIC, ehm.srv.PlanToSelectedTargetPose)
                rospy.wait_for_service(hec.EXECUTE_PLAN_TOPIC)
                self.execute_plan_proxy = rospy.ServiceProxy(hec.EXECUTE_PLAN_TOPIC, ehm.srv.ExecutePlan)

    # services: sampling

    def get_sample_list(self):
        return self.get_sample_proxy().samples

    def take_sample(self):
        return self.take_sample_proxy().samples

    def remove_sample(self, index):
        return self.remove_sample_proxy(ehm.srv.RemoveSampleRequest(sample_index=index)).samples

    # services: calibration

    def list_algorithms(self):
        return self.list_algorithms_proxy()

    def set_algorithm(self, new_algorithm):
        return self.set_algorithm_proxy(new_algorithm)

    def compute_calibration(self):
        return self.compute_calibration_proxy()

    def save(self):
        return self.save_calibration_proxy()

    # TODO: services: evaluation

    # services: robot movement
    def check_starting_pose(self):
        return self.check_starting_pose_proxy()

    def enumerate_target_poses(self):
        return self.enumerate_target_poses_proxy()

    def select_target_pose(self, i):
        return self.select_target_pose_proxy(i)

    def plan_to_selected_target_pose(self):
        return self.plan_to_selected_target_pose_proxy()

    def execute_plan(self):
        return self.execute_plan_proxy()

