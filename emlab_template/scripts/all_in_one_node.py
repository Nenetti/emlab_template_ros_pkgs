#! /usr/bin/env python
import time
from pathlib import Path
from typing import Final

import actionlib
import dynamic_reconfigure.encoding
import rospy
import std_msgs.msg as std_msgs
import emlab_template_msgs.msg as emlab_template_msgs
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from emlab_template.cfg import DynamicConfig


class AllInOneTemplateNode:

    _dynamic_reconfigure_server: Final[DynamicReconfigureServer]
    _standard_publisher: Final[rospy.Publisher]
    _latch_publisher: Final[rospy.Publisher]
    _action_client: Final[actionlib.SimpleActionClient]
    _subscriber: Final[rospy.Subscriber]
    # _action_server: Final[actionlib.SimpleActionServer]

    _sleep_time: Final[int]
    _rate: rospy.Rate
    _is_sleeping: int

    def __init__(self) -> None:
        # -------------------------------------- #
        #   1. Load rosparam & 2. Define Instance Variables
        # -------------------------------------- #
        hz = rospy.get_param("~hz")
        self._sleep_time = rospy.get_param("~sleep_time")

        self._rate = rospy.Rate(hz)
        self._is_sleeping = False

        # -------------------------------------- #
        #   3. Init Dynamic Reconfigure
        # -------------------------------------- #
        self._dynamic_reconfigure_server = DynamicReconfigureServer(DynamicConfig, self._dynamic_reconfigure_callback)

        # -------------------------------------- #
        #   4. Initialize ROS Publishers
        # -------------------------------------- #
        # self._standard_publisher = rospy.Publisher("template_sample", emlab_template_msgs.Samples, queue_size=1)
        self._standard_publisher = rospy.Publisher("template_samples", emlab_template_msgs.Samples, queue_size=1)
        self._latch_publisher = rospy.Publisher("template_latch", std_msgs.String, latch=True, queue_size=1)

        # -------------------------------------- #
        #   5 Initialize ROS Action Client
        # -------------------------------------- #
        self._action_client = actionlib.SimpleActionClient("template_sleep_action", emlab_template_msgs.SleepAction)

        # -------------------------------------- #
        #   6. Initialize ROS Action Server
        # -------------------------------------- #
        # Always "auto_start" should be "False" in constructor args.
        # self._action_server = actionlib.SimpleActionServer(..., auto_start=False)

        # -------------------------------------- #
        #   7. Wait to Start Action Server
        # -------------------------------------- #
        rospy.loginfo(f"Wait Action Server: {self._action_client.action_client.ns}")
        self._action_client.wait_for_server()

        # -------------------------------------- #
        #   8. Initialize ROS Subscriber
        # -------------------------------------- #
        self._subscriber = rospy.Subscriber("template_sample", emlab_template_msgs.Sample, self._subscriber_callback, queue_size=1)

        # -------------------------------------- #
        #   9. Start ROS Server
        # -------------------------------------- #
        # self._action_server.start()

    # ==================================================================================================================
    #
    #   Public Method
    #
    # ==================================================================================================================
    def start(self):
        """
        Main thread

        """
        rospy.loginfo("Ready")

        # Publish latch message
        latch_text = f"Connection={self._latch_publisher.get_num_connections()}, time={time.time()}"
        latch_msg = std_msgs.String(latch_text)
        self._latch_publisher.publish(latch_msg)

        # Sleep target second
        previous_step_time = time.time()
        sleep_goal = emlab_template_msgs.SleepGoal(time=std_msgs.Float32(self._sleep_time))
        self._action_client.send_goal(
            sleep_goal, done_cb=self._action_client_done_callback, feedback_cb=self._action_client_feedback_callback
        )
        self._is_sleeping = True

        while (not rospy.is_shutdown()) and self._is_sleeping:
            self._rate.sleep()

        # Test rospy.Rate using DynamicReconfigure
        while not rospy.is_shutdown():

            now_time = time.time()
            diff_time = now_time - previous_step_time
            previous_step_time = now_time

            text = f"Elapsed: {diff_time}"

            if 0 < self._standard_publisher.get_num_connections():
                samples_msg = emlab_template_msgs.Samples()
                samples_msg.values = [
                    emlab_template_msgs.Sample(),
                    emlab_template_msgs.Sample(header=std_msgs.Header(stamp=rospy.Time.now()))
                ]
                self._standard_publisher.publish(samples_msg)
                rospy.loginfo(f"Publish: {self._standard_publisher.resolved_name}")

            rospy.loginfo(f"Loop: {text}")

            self._rate.sleep()

    # ==================================================================================================================
    #
    #   [ROS Subscriber] Callbacks
    #
    # ==================================================================================================================
    def _subscriber_callback(self, msg):
        """

        Args:
            msg (emlab_template_msgs.Samples):

        Returns:

        """
        rospy.loginfo(f"Subscribe: {self._subscriber.resolved_name}")

    # ==================================================================================================================
    #
    #   [ROS Action Server] Callbacks
    #
    # ==================================================================================================================
    def _action_client_done_callback(self, status, result):
        """

        Args:
            status (int): Refer actionlib.SimpleGoalState
            result (emlab_template_msgs.SleepResult):

        Returns:

        """
        self._is_sleeping = False
        rospy.loginfo(f"Done Action Server: {self._action_client.action_client.ns}: result = {result.text.data}")

    @staticmethod
    def _action_client_feedback_callback(feedback):
        """

        Args:
            feedback (emlab_template_msgs.SleepFeedback):

        Returns:

        """
        rospy.loginfo(f"Feedback: progress = {feedback.progress.data * 100:.0f} %")

    # ==================================================================================================================
    #
    #   [ROS Dynamic Reconfigure] Callbacks
    #
    # ==================================================================================================================
    def _dynamic_reconfigure_callback(self, config, level):
        """

        Args:
            config (dynamic_reconfigure.encoding.Config):
            level (int): Refer actionlib.SimpleGoalState

        Returns:
            dynamic_reconfigure.encoding.Config:

        """
        hz: int = config.hz
        self._rate = rospy.Rate(hz)
        return config


if __name__ == '__main__':
    rospy.init_node(Path(__file__).stem)
    node = AllInOneTemplateNode()
    rospy.sleep(0.5)
    node.start()
