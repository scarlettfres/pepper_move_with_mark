#!/usr/bin/env python
import roslib
import rospy
import naoqi
import time
import sys
from math import pi
import numpy as np
from visualization_msgs.msg import Marker
from std_msgs.msg import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist # for sending commands to the drone
from naoqi import ALProxy
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# -- variables magiques 
FINESSE=0.01	# a 1 cm pres
FINESSE_ANGLE=0.01 
DURATION_LOST=0.3
TIMER_ALMOTION=0.5
HAUTEUR_ERREUR=0.5
PITCH_MAX=30	
VISI_MAX=3
HEAD_MAX_SPEED=0.08
#IP="127.0.0.1"
IP="10.0.206.111"
PORT=9559

# -- variables magiques 


def sign(x):
    if x > 0:
        return 1
    if x == 0:
        return 0
    return -1

class move_pepper:
	def __init__(self,coord,ID):
		
		self.motionProxy = ALProxy("ALMotion",IP,PORT)
		self.postureProxy = ALProxy("ALRobotPosture", IP,PORT)
		self.move=False
		tab_coord=str(coord).split()


		if len(tab_coord)==0 :
			print" no argument => no move "
			self.coord=[]
		elif len(tab_coord)==1 :	# juste tourner 
				self.coord=[float(tab_coord[0])]
		elif len(tab_coord)==2 :	# juste translation
				self.coord=[float(tab_coord[0]), float(tab_coord[1])]
		elif len(tab_coord)==3 :		# combo
				self.coord=[float(tab_coord[0]), float(tab_coord[1]), float(tab_coord[2])]
		else:
			print "to many arg"

		self.command_head=[0,0]
		self.tosend = [0,0,0]
		self.clock=rospy.Time.now()
		self.id_markeur_tete=ID
		self.listener = tf.TransformListener()
		rospy.Subscriber("/result_position", Marker,self.position_callback)
		rospy.Timer(rospy.Duration(TIMER_ALMOTION), self.timer_callback)


	def angle_to_turn(self,quaternion):
		euler=euler_from_quaternion(quaternion)	
		to_turn = self.coord[len(self.coord)-1] - euler[2]	# ce quon veut moins ce quon a 
		if abs(to_turn)<pi:	 # !!changement de signe a PI dans le repere rviz
			print "iiiiif"
			print "to turn",to_turn
			return to_turn
		else:
			print "else"
			to_turn = pi - abs(self.coord[len(self.coord)-1]) + pi - abs(euler[2])
			print "to turn",to_turn
			return to_turn*(-sign(self.coord[len(self.coord)-1]))


	def position_callback(self,data):
		print "coucou"
		
		quaternion=[data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]
		
		
		if len(self.coord) == 1:
				self.tosend = [0,0,self.angle_to_turn(quaternion)]
		elif len(self.coord) == 2:
				self.tosend = [self.coord[0]-data.pose.position.x,self.coord[1]-data.pose.position.y,0]
		elif len(self.coord) == 3:
				self.tosend = [self.coord[0]-data.pose.position.x,self.coord[1]-data.pose.position.y,self.angle_to_turn(quaternion)]


		#--- gestion de l inclinaison de la tete 
		euler=euler_from_quaternion(quaternion)		
		(trans,rot) = self.listener.lookupTransform("map", "axis_camera", rospy.Time(0))
		(trans_head_cam,rot_head_cam) = self.listener.lookupTransform("axis_camera", "mon_tf/HeadTouchFront_frame", rospy.Time(0))
		(trans_foot_cam,rot_foot_cam) = self.listener.lookupTransform("axis_camera", "mon_tf/base_link", rospy.Time(0))
		euler_head_cam = euler_from_quaternion(rot_head_cam)
		euler_foot_cam = euler_from_quaternion(rot_foot_cam)

		print euler_foot_cam
		print euler_head_cam
		#euler_cam = euler_from_quaternion(rot)

		
		print "euler[2]",euler[2]
		yawtosend = (euler_robot_cam[2]-euler[2])

		#yawtosend = euler_robot_cam[2]
		print "yawtosend",yawtosend
		if abs(yawtosend)>1.05:
				yawtosend=1.05*sign(yawtosend)
		print "yawtosend post ",yawtosend


		pitchtosend=sign(abs(euler[2])-pi/2)*(-trans[0]+data.pose.position.x)*PITCH_MAX/VISI_MAX*pi/180
		#--- gestion de l inclinaison de la tete  


		# --- on envoie la comande si pas de bug et si eloigne du goal 
		if data.pose.position.z>HAUTEUR_ERREUR :	# to avoid the bug of bad calibration that makes the robot fly 
			print "bug calib or not detected "
			self.move=False
			self.command_head=[yawtosend,pitchtosend]	# TODO

		else:
			self.command_head=[yawtosend,pitchtosend]
			if data.text == "detected":
				if (abs(self.tosend[0])>FINESSE or abs(self.tosend[1])>FINESSE or abs(self.tosend[2])>FINESSE ):
					self.move=True
					print "True"
				
				else:
					self.move=False
					#self.motionProxy.waitUntilMoveIsFinished()
					print "goal reached"
					#rospy.signal_shutdown(' goal reached ')

		"""
		if data.pose.position.z>HAUTEUR_ERREUR and data.text != "detected":	# to avoid the bug of bad calibration that makes the robot fly 
			print "bug calib or not detected "
			self.move=False
		else:
			if (abs(self.tosend[0])>FINESSE or abs(self.tosend[1])>FINESSE or abs(self.tosend[2])>FINESSE ):
				self.move=True
				print "True"
			
			else:
				self.move=False
				print "goal reached"
				rospy.signal_shutdown(' goal reached ')
		"""

	def timer_callback(self,data):
		names  = ["HeadYaw", "HeadPitch"]
		angles  = [self.command_head[0], self.command_head[1]]
		fractionMaxSpeed  = HEAD_MAX_SPEED
		self.motionProxy.setAngles(names, angles, fractionMaxSpeed)


		print "timer"
		if self.move==True : 
			print "tosend", self.tosend
			#time.wait(3)
			self.motionProxy.post.moveTo(self.tosend[0], self.tosend[1], self.tosend[2])	# sens axes repere pepper = sens axes repere rviz ICIIIIIIIIIIIIIIIIIIIIIIII
			# wait is useful because moveTo is not blocking function
			print "pre wait"
			self.motionProxy.waitUntilMoveIsFinished()
			print "post wait"

		

		
def updateArgs(arg_defaults):
    '''Look up parameters starting in the driver's private parameter space, but
    also searching outer namespaces.  '''
    args = {}
    for name, val in arg_defaults.iteritems():
        full_name = rospy.search_param(name)
        if full_name is None:
            args[name] = val
        else:
            args[name] = rospy.get_param(full_name, val)
   
    return(args)

		

def main(args):

	rospy.init_node('move_pepper', anonymous=True)
	arg_defaults = {
	'coord': "",
	'ID':"2"
	}
	args = updateArgs(arg_defaults)
	print args

	move_pepper(**args)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
		
		print "Finished."


if __name__ == '__main__':
	main(sys.argv)

