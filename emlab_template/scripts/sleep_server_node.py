#! /usr/bin/env python
import time
from pathlib import Path
from typing import Final

import actionlib
import emlab_template_msgs.msg as emlab_template_msgs
import rospy
import std_msgs.msg as std_msgs


class TemplateServerNode:

    _action_server: Final[actionlib.SimpleActionServer]

    def __init__(self) -> None:
        # -------------------------------------- #
        #   6. Initialize ROS Action Server
        # -------------------------------------- #
        # Always "auto_start" should be "False" in constructor args.
        self._action_server = actionlib.SimpleActionServer(
            "template_sleep_action", emlab_template_msgs.SleepAction, self._action_callback, auto_start=False
        )

        # -------------------------------------- #
        #   9. Start ROS Server
        # -------------------------------------- #
        self._action_server.start()

    # ==================================================================================================================
    #
    #   Public Method
    #
    # ==================================================================================================================
    @staticmethod
    def start() -> None:
        rospy.loginfo("Ready")
        rospy.spin()

    # ==================================================================================================================
    #
    #   [ROS Action Server] Callbacks
    #
    # ==================================================================================================================
    def _action_callback(self, goal) -> None:
        """

        Args:
            goal (emlab_template_msgs.SleepGoal):

        Returns:
            None:

        """
        sleep_time = goal.time.data
        start_time = time.time()
        rate = rospy.Rate(hz=100)
        while not rospy.is_shutdown():
            elapsed_time = time.time() - start_time
            if sleep_time < elapsed_time:
                break

            feedback_msg = emlab_template_msgs.SleepFeedback(progress=std_msgs.Float32((elapsed_time / sleep_time)))
            self._action_server.publish_feedback(feedback_msg)
            rate.sleep()

        result_msg = emlab_template_msgs.SleepResult(text=std_msgs.String("DONE!!!"))
        self._action_server.set_succeeded(result_msg)
        rospy.loginfo(f"Send Action Result: {self._action_server.action_server.ns}")


if __name__ == '__main__':
    rospy.init_node(Path(__file__).stem)
    node = TemplateServerNode()
    rospy.sleep(1)
    node.start()
