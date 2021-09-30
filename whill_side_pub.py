#!/usr/bin/env python

import rospy
import time
from roslibpy import Message, Ros, Topic
import tf2_ros
import geometry_msgs.msg
import tf2_msgs.msg


if __name__ == '__main__':
    rospy.init_node("talker",anonymous=True)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate=rospy.Rate(5)
    while not rospy.is_shutdown():    
    
        try:

                abc = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
                #abc = tfBuffer.lookup_transform('map', 'side', rospy.Time())
                print(abc)
                pub=rospy.Publisher('abc',geometry_msgs.msg.TransformStamped)
                
                pub.publish(abc)
                rate.sleep()
                print("abc")
        except:
            import traceback
            traceback.print_exc()
