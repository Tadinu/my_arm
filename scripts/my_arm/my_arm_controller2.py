#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

from math import sin

import roslib; roslib.load_manifest('joint_animation_tutorial')
import math, time

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import sys
import leap_interface
import Leap

twist = Twist()

def catchHandData():
    li = leap_interface.Runner()
    li.setDaemon(True)
    li.start()
    # pub     = rospy.Publisher('leapmotion/raw',leap)
    # pub_ros   = rospy.Publisher('leapmotion/data',leapros)
    rospy.init_node('leap_skeleton_pub')


    while not rospy.is_shutdown():
        timenow=rospy.Time.now()

        if li.listener.left_hand:
            analyzeHandData(li.listener.left_hand)
        else if li.listener.right_hand:
            analyzeHandData(li.listener.right_hand)

        # save some CPU time, circa 100Hz publishing.
        rospy.sleep(0.01)

def analyzeHandData(hand):
    for finger in hand.fingers:
        finger_name=hand_name+"_"+finger_names[finger.type()]

        prev_bone_name=hand_name
        for num in range(0,4):
            bone=finger.bone(num)
            bone_name=finger_name+"_"+bones_names[num]

            prev_bone_name=bone_name
            prev_bone_absolute=bone_absolute

    jointStatesCommand(fingers)

def jointStatesCommand(jointValueMsg):
    pub = rospy.Publisher('/joint_states', jointValueMsg, queue_size=10)

def jointTrajectoryCommand():
    # Initialize the node
    rospy.init_node('joint_control')

    print rospy.get_rostime().to_sec()
    while rospy.get_rostime().to_sec() == 0.0:
        time.sleep(0.1)
        print rospy.get_rostime().to_sec()

    pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)
    jt = JointTrajectory()

    jt.header.stamp = rospy.Time.now()
    jt.header.frame_id = "atlas::pelvis"

    jt.joint_names.append("atlas::back_lbz" )
    jt.joint_names.append("atlas::back_mby" )
    jt.joint_names.append("atlas::back_ubx" )
    jt.joint_names.append("atlas::neck_ay"  )
    jt.joint_names.append("atlas::l_leg_uhz")
    jt.joint_names.append("atlas::l_leg_mhx")
    jt.joint_names.append("atlas::l_leg_lhy")
    jt.joint_names.append("atlas::l_leg_kny")
    jt.joint_names.append("atlas::l_leg_uay")
    jt.joint_names.append("atlas::l_leg_lax")
    jt.joint_names.append("atlas::r_leg_lax")
    jt.joint_names.append("atlas::r_leg_uay")
    jt.joint_names.append("atlas::r_leg_kny")
    jt.joint_names.append("atlas::r_leg_lhy")
    jt.joint_names.append("atlas::r_leg_mhx")
    jt.joint_names.append("atlas::r_leg_uhz")
    jt.joint_names.append("atlas::l_arm_elx")
    jt.joint_names.append("atlas::l_arm_ely")
    jt.joint_names.append("atlas::l_arm_mwx")
    jt.joint_names.append("atlas::l_arm_shx")
    jt.joint_names.append("atlas::l_arm_usy")
    jt.joint_names.append("atlas::l_arm_uwy")
    jt.joint_names.append("atlas::r_arm_elx")
    jt.joint_names.append("atlas::r_arm_ely")
    jt.joint_names.append("atlas::r_arm_mwx")
    jt.joint_names.append("atlas::r_arm_shx")
    jt.joint_names.append("atlas::r_arm_usy")
    jt.joint_names.append("atlas::r_arm_uwy")

    n = 1500
    dt = 0.01
    rps = 0.05
    for i in range (n):
        p = JointTrajectoryPoint()
        theta = rps*2.0*math.pi*i*dt
        x1 = -0.5*math.sin(2*theta)
        x2 =  0.5*math.sin(1*theta)

        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x1)
        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x1)
        p.positions.append(x2)
        p.positions.append(x1)
        p.positions.append(x1)
        jt.points.append(p)

        # set duration
        jt.points[i].time_from_start = rospy.Duration.from_sec(dt)
        rospy.loginfo("test: angles[%d][%f, %f]",n,x1,x2)

    pub.publish(jt)
    rospy.spin()

# Main Function
def main():
    rospy.init_node('robotArmControllerPy')
    print rospy.get_rostime().to_sec()
    while rospy.get_rostime().to_sec() == 0.0:
        time.sleep(0.1)
        print rospy.get_rostime().to_sec()

    catchHandData()

def values():
    print '(w for forward, a for left, s for reverse, d for right,k for turning left,l for turning right and . to exit)' + '\n'
    s = raw_input(':- ')
    if s[0] == 'w':
        twist.linear.x = 1.0
        twist.angular.z = 0.0
        twist.linear.y = 0.0
    elif s[0] == 's':
        twist.linear.x = -1.0
        twist.angular.z = 0.0
        twist.linear.y = 0.0
    elif s[0] == 'd':
        twist.linear.y = -1.0
        twist.angular.z = 0.0
        twist.linear.x = 0.0
    elif s[0] == 'a':
        twist.linear.y = 1.0
        twist.angular.z = 0.0
        twist.linear.x = 0.0
    elif s[0] == 'k':
        twist.angular.z = 2.0
        twist.linear.x = twist.linear.y = 0.0
    elif s[0] == 'l':
        twist.angular.z = -2.0
        twist.linear.x = twist.linear.y = 0.0
    elif s[0] == '.':
        twist.angular.z = twist.linear.x = twist.linear.y = 0.0
        sys.exit()
    else:
        twist.linear.x = twist.linear.y = twist.angular.z = 0.0
        print 'Wrong command entered \n'
    return twist

def keyboard():
    pub = rospy.Publisher('base_controller/command',Twist, queue_size=1)
    rospy.init_node('teleop_py',anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        twist = values()
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
