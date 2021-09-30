import rospy
import smach
from pocketsphinx_jsgf import PocketSphinxClient
from common.speech import DefaultTTS
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

class FinishMoving(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['finish', 'continue', 'failure'],
                             input_keys=['robot_poses'],
                             output_keys=['robot_poses'])
        self._sphinx = PocketSphinxClient()
        self._tts = DefaultTTS()
        self._gripper = robot.get('gripper')
        self._whole_body = robot.get('whole_body')
	self._omni_base = robot.get("omni_base")

    def execute(self, userdata):
	try:
		self._tts.say('Should I stop here?')
		count = 0
		while not rospy.is_shutdown():
		    ans = self._sphinx.next_yes_or_no()
		    if ans == 'yes':
		        self._tts.say("OK. I will learn the position.")
		        
		        #pose_with_cov = rospy.wait_for_message('/laser_2d_pose', PoseWithCovarianceStamped)
		        #pos = (pose_with_cov.pose.pose.position.x, pose_with_cov.pose.pose.position.y)
		        #q = pose_with_cov.pose.pose.orientation
		        #angle = np.arctan2(q.z, q.w) * 2

			pose = self._omni_base.pose
			pos = (pose[0], pose[1])
			angle = pose[2]

			robot_pose = (pos, angle)

			userdata.robot_poses.append(robot_pose)

			rospy.loginfo('robot_pose: {}'.format( robot_pose ) )
		        try:
		            with open('/tmp/robot_pose', 'w') as f:
		                f.writelines([str(robot_pose)])
		        except:
		            pass
		        
		        return 'finish'
		    if ans == 'no':
		        self._tts.say("OK. Let's continue moving.")
		        return 'continue'

		    count += 1
		    if count > 5:
		        return 'failure'
	except:
		return 'failure'

class FinishMovingNoPose(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['finish', 'continue', 'failure'])
        self._sphinx = PocketSphinxClient()
        self._tts = DefaultTTS()
        self._gripper = robot.get('gripper')
        self._whole_body = robot.get('whole_body')
	self._omni_base = robot.get("omni_base")

    def execute(self, userdata):
	try:
		self._tts.say('Should I stop here?')
		count = 0
		while not rospy.is_shutdown():
		    ans = self._sphinx.next_yes_or_no()
		    if ans == 'yes':
		        self._tts.say("OK.")
		        
		        
		        return 'finish'
		    if ans == 'no':
		        self._tts.say("OK. Let's continue moving.")
		        return 'continue'

		    count += 1
		    if count > 5:
		        return 'failure'
	except:
		return 'failure'

class FinishMovingNoConfirmation(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['finish', 'failure'],
                             input_keys=['robot_poses'],
                             output_keys=['robot_poses'])
        self._tts = DefaultTTS()
        self._gripper = robot.get('gripper')
        self._whole_body = robot.get('whole_body')
	self._omni_base = robot.get("omni_base")

    def execute(self, userdata):
	try:
	        self._tts.say("OK. I will learn the position.")
	        
	        #pose_with_cov = rospy.wait_for_message('/laser_2d_pose', PoseWithCovarianceStamped)
	        #pos = (pose_with_cov.pose.pose.position.x, pose_with_cov.pose.pose.position.y)
	        #q = pose_with_cov.pose.pose.orientation
	        #angle = np.arctan2(q.z, q.w) * 2

		pose = self._omni_base.pose
		pos = (pose[0], pose[1])
		angle = pose[2]

		robot_pose = (pos, angle)

		userdata.robot_poses.append(robot_pose)

		rospy.loginfo('robot_pose: {}'.format( robot_pose ) )
	        try:
	            with open('/tmp/robot_pose', 'w') as f:
	                f.writelines([str(robot_pose)])
	        except:
	            pass
		        
	        return 'finish'

	except:
		return 'failure'

# unit test
if __name__=='__main__':
    import hsrb_interface
    #rospy.init_node('finish_carrying_unit_test')
    robot = hsrb_interface.Robot()
    sm = smach.StateMachine(outcomes=['finish', 'continue', 'failure'])
    with sm:
        smach.StateMachine.add('FinishMoving', FinishMoving(robot))
    print 'Result:', sm.execute()
    print 'usertada: ', dict(sm.userdata)
