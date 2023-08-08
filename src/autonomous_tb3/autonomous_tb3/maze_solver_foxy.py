import sys

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
# from robot_navigator import BasicNavigator, TaskResult
from autonomous_tb3.robot_navigator_foxy import BasicNavigator
import rclpy
from rclpy.duration import Duration

from math import *
import tf_transformations


"""
Basic navigation demo to go to pose.
"""


def main(argv=sys.argv[1:]):
    rclpy.init()
    navigator = BasicNavigator()

    # Converts quaternion (w in last place) to euler roll, pitch, yaw
    # quaternion = [w, x, y, z]
    # Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    # q_w, q_x, q_y, q_z = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)

    # Set our demo's initial pose
    initial_pose = Pose()
    initial_pose.position.x = -3.0
    initial_pose.position.y = -7.0
    initial_pose.orientation.w = q_w
    initial_pose.orientation.x = q_x
    initial_pose.orientation.y = q_y
    initial_pose.orientation.z = q_z
    
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()
    # Define a list of goal coordinates
    goal_coordinates = [
        # # 단일 성공 케이스 1
        # (0.0, 0.0, 0.0),  # x, y, theta for Goal 1

        # # 단일 성공 케이스 2
        # (-1.0, -2.0, 0.0),  # x, y, theta for Goal 1

        # # 단일 성공 케이스 3
        # (-7.6, -0.9, 0.0),  # x, y, theta for Goal 1

        # # 멀티 성공 케이스 4
        # (0.0, 0.0, -pi/2),  # x, y, theta for Goal 1
        # (8.0, -2.0, 0.0)   # x, y, theta for Goal 2

        # # 멀티 성공 케이스 5
        # (-1.0, -2.0, pi),  # x, y, theta for Goal 1
        # (8.0, -2.0, pi)   # x, y, theta for Goal 2 # 성공

        # # 멀티 성공 케이스 1
        # (-1.0, -2.0, pi),  # x, y, theta for Goal 1
        # (-7.6, -0.9, 0.0),  # x, y, theta for Goal 1
        # (8.0, -2.0, 0.0)   # x, y, theta for Goal 2

        # 멀티 성공 케이스 1
        # (-1.0, -2.0, pi),  # x, y, theta for Goal 1
        # (-7.6, -0.9, pi/2.0),  # x, y, theta for Goal 1
        # (-7.19, 7.67, 0.0),
        # (-0.29, 5.78, -pi/2.0),
        # (8.0, -2.0, 0.0)   # x, y, theta for Goal 2
        
        (-1.0, -2.0, -3.0*pi/4.0),  # x, y, theta for Goal 1
        (-7.45, -3.02, pi/2.0),  # x, y, theta for Goal 1
        (-7.0, 7.28, 0.0),
        (-0.56, 3.37, 0.0),
        (8.0, -2.0, 0.0),   # x, y, theta for Goal 2
    ]

    for goal_x, goal_y, goal_theta in goal_coordinates:
        # Create a goal message with the PoseStamped message
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        # q_w, q_x, q_y, q_z = tf_transformations.quaternion_from_euler(0.0, 0.0, goal_theta)
        q_x, q_y, q_z, q_w  = tf_transformations.quaternion_from_euler(0.0, 0.0, goal_theta)
        goal_pose.pose.orientation.x = q_x
        goal_pose.pose.orientation.y = q_y
        goal_pose.pose.orientation.z = q_z
        goal_pose.pose.orientation.w = q_w

        # Go to the current goal pose
        navigator.goToPose(goal_pose)

        i = 0
        while not navigator.isNavComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                # print('Estimated time of arrival: ' + '{0:.0f}'.format(
                #       Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                #       + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelNav()

        # Do something depending on the return code
        result = navigator.getResult()
        if result == GoalStatus.STATUS_SUCCEEDED:
            print('Goal succeeded!')
        elif result == GoalStatus.STATUS_CANCELED:
            print('Goal was canceled!')
        elif result == GoalStatus.STATUS_ABORTED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

    exit(0)

if __name__ == '__main__':
    main()