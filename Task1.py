#!/usr/bin/env python
import math
import rosbag
import numpy as np
from Relative2AbsolutePose import Relative2AbsolutePose
from Relative2AbsoluteXY import Relative2AbsoluteXY
from cly_rel2abs import cly_rel2abs
from cv_bridge import CvBridge, CvBridgeError
import sys,select,termios, tty
import time
import rospy
from cylinder.msg import cylDataArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

global id_pose, id_odom, id_landmark
id_pose = 0
id_odom = 0
id_landmark = 0

global robot_abs
robot_abs = [0, 0, 0]
global time2,p,counter
time2 = 0.0
p = 0.0 #period of timesteps
counter = 0

def callback_tel(data_tel):
	global id_pose,robot_abs,p,counter
	if counter%1 == 0:
	    u = [data_tel.linear.x*p,data_tel.linear.y*p,data_tel.angular.z*p]
	    robot_abs = Relative2AbsolutePose(robot_abs,u)
	    line = ("POSE2D "+ str(id_pose) + " "+ str(robot_abs[0]) + " " +str(robot_abs[1]) + " " + str(robot_abs[2]) + " " + "\n")
	    id_pose = id_pose + 1
	    point_file.write(line)
	    #print line
	    
	counter = counter +1
	#print counter

def callback_cly(data_cly):
	global robot_abs, id_landmark

	if len(data_cly.cylinders) != 0:
		for i in range(len(data_cly.cylinders)):
            #Xrobot is the x position of cylinder and Zrobot is the depth
			cly_dis = np.array([data_cly.cylinders[i].Xrobot, data_cly.cylinders[i].Zrobot])
			s_xx = data_cly.cylinders[i].covariance[0]
			s_xy = data_cly.cylinders[i].covariance[1]
			s_yy = data_cly.cylinders[i].covariance[3]
			
			line = ("LANDMARK_MEAS2D " + str(id_landmark) + " " + str(cly_dis[0]) + " " + str(cly_dis[1]) + " " 
			+ str(s_xx) + " " + str(s_xy) + " " + str(s_yy) + "\n")
			
			measurement_file.write(line)
			#print(line)
			
			cly_abs = Relative2AbsoluteXY(robot_abs,cly_dis)
			line2 = ("POINT2D " + str(id_landmark) + " " + str(cly_abs[0]) + " " + str(cly_abs[1]) + "\n")
			#print line2
			point_file.write(line2)
			id_landmark = id_landmark + 1 
            
                             
def callback_odm(data_odm):
	global time2,p,id_odom
	time1 = time2
	time2 = data_odm.header.stamp.secs + data_odm.header.stamp.nsecs * (10 ** -9)
	p = time2 - time1
	
	odom_meas = [data_odm.pose.pose.position.x,data_odm.pose.pose.position.y]
	q0 = data_odm.pose.pose.orientation.x #qo
	q1 = data_odm.pose.pose.orientation.y #q1
	q2 = data_odm.pose.pose.orientation.z #q2
	q3 = data_odm.pose.pose.orientation.w #q3
	
	theta = math.atan2(2*(q0*q1+q2*q3),(1-2*(q1**2+q2**2)))
	
	if theta<0:
	    theta = theta +2*np.pi
	odom_xy = " ".join(str(x) for x in odom_meas)

	variances = data_odm.pose.covariance
	s_xx = data_odm.pose.covariance[0]
	s_xy = data_odm.pose.covariance[1]
	s_xtheta = data_odm.pose.covariance[5]
	s_yy = data_odm.pose.covariance[7]
	s_ytheta = data_odm.pose.covariance[11]
	s_theta2 = data_odm.pose.covariance[35]
	
	line = ("ODOMETRY_MEAS2D " + str(id_odom) + " "+ str(odom_xy) + " " + str(theta) + " " + str(s_xx) + " " + str(s_xy)
	+ " " + str(s_xtheta) + " "+ str(s_yy) + " " + str(s_ytheta) + " " + str(s_theta2) + "\n")
	#print s_xx, s_xy, s_xtheta, "\n"
	#print s_yy, s_ytheta,s_theta2, "\n"
	#print time2
	measurement_file.write(line)
	id_odom = id_odom +1

	
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('task2_node', anonymous=True)
    point_file = open('/home/wenlin/catkin_ws/src/cylinder/src/Tragectory and Points.txt', 'w')
    measurement_file = open('/home/wenlin/catkin_ws/src/cylinder/src/Measurements.txt','w')
    data_odm = rospy.Subscriber("/odom",Odometry,callback_odm)
    data_tel = rospy.Subscriber("/cmd_vel_mux/input/teleop",Twist,callback_tel)
    data_cly = rospy.Subscriber("/cylinderTopic",cylDataArray,callback_cly)
    
    while(1):
	key = getKey()
	if key == 'q': 
		break
		

    point_file.close()
    measurement_file.close()

