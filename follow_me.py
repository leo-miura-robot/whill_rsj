#!/usr/bin/env python
import actionlib
import rospy
import time
from roslibpy import Message, Ros, Topic
import tf2_msgs.msg
import tf
from hsrb_interface import Robot
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import tf.transformations

_MOVE_TIMEOUT=60.0
robot = Robot()
omni_base = robot.get("omni_base")
cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

class rosbridge_client:
    def __init__(self):
        self.ros_client = Ros('192.168.195.174', 9090)
        self.listener = Topic(self.ros_client, '/abc', 'geometry_msgs/TransformStamped')
        self.listener.subscribe(self.callback)
        self.ros_client.run_forever()

    def callback(self,message):

        cli.cancel_goal()
        tf_x=message["transform"]["translation"]["x"]
        tf_y=message["transform"]["translation"]["y"]
        tf_z=message["transform"]["translation"]["z"]

        # input goal pose
        goal_x = tf_x
        goal_y = tf_y 
        goal_yaw = tf_z
        print(tf_x,tf_y)
        # fill ROS message
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = Point(goal_x, goal_y, 0)
        quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
        pose.pose.orientation = Quaternion(*quat)

        goal = MoveBaseGoal()
        goal.target_pose = pose

        # send message to the action server
        cli.send_goal(goal)
	#iranai_kamo
	rospy.sleep(0.5)


if __name__ == '__main__':
    print("start")
    rosbridge_client()
