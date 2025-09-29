from tf2_ros import Buffer, TransformListener, TransformBroadcaster
try:
    # ROS2
    from rclpy.time import Time, Duration
    from rclpy.logging import get_logger
    ROS2_AVAILABLE = True
except ImportError:
    # ROS1
    from rospy import Time, Duration, loginfo
    ROS2_AVAILABLE = False


class HandeyeSampler(object):
    """
    Manages the samples acquired from tf.
    """

    def __init__(self, handeye_parameters, node=None):
        self.handeye_parameters = handeye_parameters
        self.node = node

        # tf structures
        if node is not None:  # ROS2
            self.tfBuffer = Buffer()
            """
            used to get transforms to build each sample

            :type: tf2_ros.Buffer
            """
            self.tfListener = TransformListener(self.tfBuffer, node)
            """
            used to get transforms to build each sample

            :type: tf2_ros.TransformListener
            """
            self.tfBroadcaster = TransformBroadcaster(node)
            """
            used to publish the calibration after saving it

            :type: tf2_ros.TransformBroadcaster
            """
        else:  # ROS1
            self.tfBuffer = Buffer()
            """
            used to get transforms to build each sample

            :type: tf2_ros.Buffer
            """
            self.tfListener = TransformListener(self.tfBuffer)
            """
            used to get transforms to build each sample

            :type: tf2_ros.TransformListener
            """
            self.tfBroadcaster = TransformBroadcaster()
            """
            used to publish the calibration after saving it

            :type: tf2_ros.TransformBroadcaster
            """

        # internal input data
        self.samples = []
        """
        list of acquired samples

        Each sample is a dictionary going from 'rob' and 'opt' to the relative sampled transform in tf tuple format.

        :type: list[dict[str, ((float, float, float), (float, float, float, float))]]
        """

    def _wait_for_tf_init(self):
        """
        Waits until all needed frames are present in tf.

        :rtype: None
        """
        if self.node is not None:  # ROS2
            self.tfBuffer.lookup_transform(self.handeye_parameters.robot_base_frame,
                                           self.handeye_parameters.robot_effector_frame, Time(),
                                           Duration(seconds=20))
            self.tfBuffer.lookup_transform(self.handeye_parameters.tracking_base_frame,
                                           self.handeye_parameters.tracking_marker_frame, Time(),
                                           Duration(seconds=60))
        else:  # ROS1
            self.tfBuffer.lookup_transform(self.handeye_parameters.robot_base_frame,
                                           self.handeye_parameters.robot_effector_frame, Time(0),
                                           Duration(20))
            self.tfBuffer.lookup_transform(self.handeye_parameters.tracking_base_frame,
                                           self.handeye_parameters.tracking_marker_frame, Time(0),
                                           Duration(60))

    def _get_transforms(self, time=None):
        """
        Samples the transforms at the given time.

        :param time: sampling time (now if None)
        :type time: None|Time
        :rtype: dict[str, ((float, float, float), (float, float, float, float))]
        """
        if time is None:
            if self.node is not None:  # ROS2
                time = Time()
            else:  # ROS1
                time = Time.now()

        # here we trick the library (it is actually made for eye_on_hand only). Trust me, I'm an engineer
        if self.handeye_parameters.eye_on_hand:
            rob = self.tfBuffer.lookup_transform(self.handeye_parameters.robot_base_frame,
                                                 self.handeye_parameters.robot_effector_frame, time,
                                                 Duration(seconds=10))
        else:
            rob = self.tfBuffer.lookup_transform(self.handeye_parameters.robot_effector_frame,
                                                 self.handeye_parameters.robot_base_frame, time,
                                                 Duration(seconds=10))
        opt = self.tfBuffer.lookup_transform(self.handeye_parameters.tracking_base_frame,
                                             self.handeye_parameters.tracking_marker_frame, time,
                                             Duration(seconds=10))
        return {'robot': rob, 'optical': opt}

    def take_sample(self):
        """
        Samples the transformations and appends the sample to the list.

        :rtype: None
        """
        if self.node is not None:  # ROS2
            self.node.get_logger().info("Taking a sample...")
            transforms = self._get_transforms()
            self.node.get_logger().info("Got a sample")
        else:  # ROS1
            loginfo("Taking a sample...")
            transforms = self._get_transforms()
            loginfo("Got a sample")
        self.samples.append(transforms)

    def remove_sample(self, index):
        """
        Removes a sample from the list.

        :type index: int
        :rtype: None
        """
        if 0 <= index < len(self.samples):
            del self.samples[index]

    def get_samples(self):
        """
        Returns the samples accumulated so far.
        :rtype: [dict[str, ((float, float, float), (float, float, float, float))]]
        :return: A list of tuples containing the tracking and the robot transform pairs
        """
        return self.samples
