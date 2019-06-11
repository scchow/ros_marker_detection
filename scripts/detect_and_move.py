#!/usr/bin/env python
#
#  Copyright 2019 Connor Yates
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.
#

import rospy
import actionlib
import tf2_ros

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool, Header


def heading(yaw):
    """ A helper function to getnerate quaternions from yaws."""
    q = quaternion_from_euler(0, 0, yaw)
    return Quaternion(*q)


def go_to_goal(goal):
    """
    Takes in a single goal. Will be published and watched in the action-server.
    Goal will be provided to action server in the map frame.
    :param goal: (Pose) The pose the turtlebot will try to navigate to
    :return: status dependent on the success of the navigation
    :raises: TypeError - if goal is not a Pose
    """
    if type(goal) is not Pose:
        raise TypeError("Goal must be of type Pose.")

    mb_goal = MoveBaseGoal()
    mb_goal.target_pose.header.frame_id = 'april_tag'
    mb_goal.target_pose.header.stamp = rospy.Time.now()
    mb_goal.target_pose.pose = goal

    move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_base.wait_for_server()

    move_base.send_goal(mb_goal)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        state = move_base.get_state()
        if state not in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
            break
        rate.sleep()

    success = move_base.get_result()
    if success and state == GoalStatus.SUCCEEDED:
        return True
    else:
        move_base.cancel_all_goals()
    return False


class NavToGoal:
    def __init__(self):
        self.in_position = False
        self.in_callback = False

    def move_to_goal(self, msg):
        """
        Moves to the goal once found
        :return:
        """
        if self.in_callback or self.in_position:
            return
        self.in_callback = True
        rospy.loginfo("Starting to move to goal")
        # send goal
        for frame_id in ["switch_1", "switch_2"]:
            try:
                h = Header()
                h.frame_id = frame_id
                src_point = PoseStamped(
                        h,
                        Pose(
                            Point(0.3, 0, 0),
                            heading(180)
                            )
                        )
                tf_buf = tf2_ros.Buffer()
                tf_listner = tf2_ros.TransformListner(tf_buf)
                target_pt = tf_buf.transform(src_point, "map")
            except:
                rospy.logwarn("Skipping tag '{}'".format(frame_id))
                continue

        rospy.loginfo("Moving toward point...")
        if not go_to_goal(src_point):
            return 
        rospy.loginfo("Exiting move_to_goal callback")
        # report in-position to all robots
        self.in_position = True
        self.in_callback = False


if __name__ == '__main__':
    rospy.init_node("move_to_goal")
    nav = NavToGoal()
    rospy.Subscriber('/switch_found', Bool, nav.move_to_goal)

    rospy.spin()
