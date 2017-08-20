#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import Twist

vel = Twist()

def callback(msg1):
    alt=msg1.altd
    global vel
    ref=1500
    kp=0.55
    ki=0
    kd=0.25
    E=0
    d=0
    Eold=0
    dv=800
    e=ref-alt
    E=E+e
    d=e-Eold
    vel.linear.z = (kp*e + ki*E +kd*d)/dv
    Eold = e

def altholdpy():
    rospy.init_node('altholdpy', anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
    takeoff = rospy.Publisher("/ardrone/takeoff",Empty,queue_size=100,latch=True)
    rospy.Subscriber("/ardrone/navdata", Navdata, callback)

    rate = rospy.Rate(10)
    global vel
    blank = Empty()
    takeoff.publish(blank)

    while not rospy.is_shutdown():
        pub.publish(vel)
        rospy.loginfo(vel.linear.z)
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    try:
        altholdpy()
    except rospy.ROSInterruptException:
        pass
