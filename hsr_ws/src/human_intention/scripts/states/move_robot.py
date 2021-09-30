import rospy
import tf2_ros
import smach
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped, Vector3Stamped, Twist
from common.smach_states import TemporarySubscriber
from common.speech import DefaultTTS

class MoveRobot(smach.State):
    def __init__(self, tf_buffer=None):
        smach.State.__init__(self, outcomes=['finishing', 'failure'])

        self._arm_pub = rospy.Publisher('/hsrb/arm_trajectory_controller/command',
                                        JointTrajectory, queue_size=1)
        self._vel_pub = rospy.Publisher('/hsrb/command_velocity',
                                        Twist, queue_size=1)
        self._arm_sub = TemporarySubscriber('/hsrb/arm_trajectory_controller/state',
                                            JointTrajectoryControllerState, self._arm_callback)
        self._wrist_sub = TemporarySubscriber('/hsrb/wrist_wrench/compensated',
                                              WrenchStamped, self._wrist_callback)
        self._tts = DefaultTTS()
        if tf_buffer is None:
            tf_buffer = tf2_ros.Buffer()
            tf2_ros.TransformListener(tf_buffer)
        self._tf_buffer = tf_buffer

    def execute(self, userdata):
        # parameters
        state_threshold = 2.5
        linear_weight = .01
        angular_weight = .05
        lift_weight = .01
        lift_up_threshold = .15
        put_down_threshold = .05
        stop_threshold = 2.

        # variables
        lift_highest = 0.
        t0 = None
        self._joint_positions = None
        self._force = np.zeros(3)
        self._linear_velocity = 0.
        self._angular_velocity = 0.
        self._init_schmitt_trigger()

        self._tts.say("OK. Let's roll!")
        
        rate = rospy.Rate(10)
        with self._arm_sub, self._wrist_sub:
            while not rospy.is_shutdown():
                rate.sleep()
                if self._joint_positions is None:
                    continue

                # check the arm height to detect the finish
                joint_ind = self._joint_names.index('arm_lift_joint')
                arm_lift = self._joint_positions[joint_ind]
                lift_highest = max(lift_highest, arm_lift)
                if lift_highest > lift_up_threshold and arm_lift < put_down_threshold:
                   #and self._state == 0:
                    if t0 is not None:
                        if (rospy.Time.now() - t0).to_sec() > stop_threshold:
                            return 'finishing'
                    else:
                        t0 = rospy.Time.now()
                else:
                    t0 = None

                rospy.loginfo('arm lift position: %f' % arm_lift)
                rospy.loginfo('force: {}'.format(self._force))
                
                # move omni base or arm lift joint according to the wrist wrench
                state = self._update_schmitt_trigger([state_threshold,
                                                      abs(self._force[0]),
                                                      abs(self._force[1]),
                                                      abs(self._force[2])])
                v = 0.
                a = 0.
                l = 0.
                if state == 1:
                    v = linear_weight * self._force[0]
                    a = 0.
                elif state == 2:
                    v = 0.
                    a = angular_weight * self._force[1]
                elif state == 3:
                    l = lift_weight * self._force[2]
                self._move_omni_base(v, a)
                self._move_arm_lift_joint(l)
            return 'failure'

    def _arm_callback(self, msg):
        self._joint_names = msg.joint_names
        self._joint_positions = msg.actual.positions
    
    def _wrist_callback(self, msg):
        # parameters
        force_decay = 0.5
        
        vec = Vector3Stamped()
        vec.header = msg.header
        vec.vector = msg.wrench.force
        try:
            base = self._tf_buffer.transform(vec, 'base_footprint', rospy.Duration(10))
        except:
            rospy.logerr("TF error")

        # low-pass temporal filter
        force = np.array([base.vector.x, base.vector.y, base.vector.z])
        self._force = force_decay * self._force + (1-force_decay) * force

    def _init_schmitt_trigger(self):
        self._state_prob = np.zeros(4)
        self._state = 0
        
    def _update_schmitt_trigger(self, values):
        # parameters
        schmitt_decay = .8
        schmitt_threshold = .8

        s = np.zeros(4)
        s[np.argmax(values)] = 1.
        
        # low-pass temporal filter
        self._state_prob = schmitt_decay * self._state_prob + (1-schmitt_decay) * s
        
        self._state_prob /= self._state_prob.sum()
        if self._state_prob.max() > schmitt_threshold:
            self._state = self._state_prob.argmax()
        return self._state
    
    def _move_omni_base(self, linear, angular):
        # parameters
        linear_decay = 0.6
        angular_decay = 0.6
        
        # low-pass temporal filter
        self._linear_velocity = linear_decay * self._linear_velocity + (1-linear_decay) * linear
        self._angular_velocity = angular_decay * self._angular_velocity + (1-angular_decay) * angular
        
        twist = Twist()
        twist.linear.x = self._linear_velocity
        twist.angular.z = self._angular_velocity
        self._vel_pub.publish(twist)
        
    def _move_arm_lift_joint(self, dx):
        joint_ind = self._joint_names.index('arm_lift_joint')
        traj = JointTrajectoryPoint()
        traj.positions = list(self._joint_positions)
        traj.positions[joint_ind] += dx
        traj.time_from_start = rospy.Duration(1.)
        self._arm_pub.publish(JointTrajectory(joint_names=self._joint_names, points=[traj]))
        
# unit test
if __name__=='__main__':
    rospy.init_node('carry_table_unit_test')
    sm = smach.StateMachine(outcomes=['finishing', 'failure'])
    with sm:
        smach.StateMachine.add('MoveTable', MoveTable())
    print 'Result:', sm.execute()

