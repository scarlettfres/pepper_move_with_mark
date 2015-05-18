#!/usr/bin/env python
import roslib
import rospy
import naoqi
import time
import sys
from math import pi,atan,sqrt,cos,sin
import numpy as np
from nav_msgs.msg import Odometry
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
TIMER_ALMOTION=0.8
HAUTEUR_ERREUR=0.5
PITCH_MAX=30	
VISI_MAX=3
temps_erreur=1
HEAD_MAX_SPEED=0.08
#IP="127.0.0.1"
IP="10.0.206.111"
PORT=9559
BACK=1
FRONT=-1
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
		self.detection=0
		self.id_markeur_tete=int(ID)
		self.clock=rospy.Time.now()

		self.listener = tf.TransformListener()
		self.broadcaster = tf.TransformBroadcaster()
		rospy.Subscriber("/result_position", Odometry,self.position_callback)
		rospy.Timer(rospy.Duration(TIMER_ALMOTION), self.timer_callback)
		rospy.Subscriber("/cam0/visualization_marker", Marker,self.mark_callback)



	def mark_callback(self,data):
		print "dddd", data.id
		print "idmarkeur", self.id_markeur_tete

		if data.id==self.id_markeur_tete:
			self.clock = rospy.Time.now() + rospy.Duration(temps_erreur)
			self.detection=1
			print "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
			print "mark callb 1"
		elif rospy.Time.now()> self.clock:
			self.detection=0
			print " markclb 0"


	def angle_to_turn(self,quaternion):
		euler=euler_from_quaternion(quaternion)	
		to_turn = self.coord[len(self.coord)-1] - euler[2]	# ce quon veut moins ce quon a 
		if abs(to_turn)<pi:	 # !!changement de signe a PI dans le repere rviz
			return to_turn
		else:
			
			to_turn = pi - abs(self.coord[len(self.coord)-1]) + pi - abs(euler[2])
			return to_turn*(-sign(self.coord[len(self.coord)-1]))



	def angle_goal_to_map(self,position,euler):

		# 1-quel est langle entre le goal et la map? 
		if len(self.coord) > 1:	# on a entre un x,y
			x=self.coord[0]
			y=self.coord[1]



			if x!=0:
				angle_map_goal = atan(y/x)
				if x<0:
					angle_map_goal = (pi-abs(angle_map_goal))*(-sign(angle_map_goal))
			else:
				angle_map_goal=pi/2
			print "amp", angle_map_goal
			# 2-quel est langle de la position actuelle du robot/ map ? => euler + angle_position_robot 
			

			if position[0]!=0:
				angle_position_robot = atan(position[1]/position[0])
				if position[0]<0:
					angle_position_robot  = (pi-abs(angle_position_robot ))*(-sign(angle_position_robot ))
			else:
				angle_position_robot =pi/2

			angle_robot_map_fin=angle_position_robot+euler[2]

			print "hoohooooh", angle_position_robot+euler[2]




			# 3- quel angle entre robot/goal ?
			print "angle fin oldd", angle_map_goal - euler[2]
			final_angle =angle_map_goal + angle_robot_map_fin

			print "fff", final_angle
			return final_angle





	def position_callback(self,data):
		print "callback"
		if len(self.coord) > 1:	# on a entre un x,y
			
			self.broadcaster.sendTransform([self.coord[0],self.coord[1],0],[0,0,0,1],rospy.Time.now(),"/mon_tf/mygoal","/map")
			(trans,rot)=self.listener.lookupTransform("/base_footprint","/mon_tf/mygoal", rospy.Time(0))
			print "trans", trans
			print "euler", euler_from_quaternion(rot)


			"""
			quaternion_robot_map=[data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
			euler = euler_from_quaternion(quaternion_robot_map)
			angle_goal =self.angle_goal_to_map(position,euler)	# on obtient l'angle relatif entre le robot et le goal 

			# translater jusqua la position : on traduit langle relatif en coord x,y relatives 
			# quelle est la distance entre le robot et le goal? 
			
			dist_rel = sqrt( pow(self.coord[0]-data.pose.pose.position.x,2) + pow(self.coord[1]-data.pose.pose.position.y,2) )
			print "dist_rel= " , dist_rel
			print "cos(angle_goal)",cos(angle_goal)
			print "sin(angle_goal)",sin(angle_goal)
			rel_x=cos(angle_goal)*dist_rel
			rel_y=sin(angle_goal)*dist_rel

			print "rel_x",rel_x
			print "rel_y",rel_y
			"""
			self.tosend = [trans[0],trans[1],0]

			if data.pose.pose.position.z>HAUTEUR_ERREUR :	# to avoid the bug of bad calibration that makes the robot fly 
				print "bug calib or not detected "
				self.move=False
				#self.command_head=[yawtosend,pitchtosend]	# TODO

			else:
				#self.command_head=[yawtosend,pitchtosend]
				if (abs(self.tosend[0])>FINESSE or abs(self.tosend[1])>FINESSE or abs(self.tosend[2])>FINESSE ):
					self.move=True
					print "True"
					
				else:
					self.move=False
					self.motionProxy.waitUntilMoveIsFinished()
					print "goal reached"
					rospy.signal_shutdown(' goal reached ')
			








	"""
	def position_callback(self,data):
		print "callback"
		quaternion=[data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
		euler=euler_from_quaternion(quaternion)		


		if abs(euler[2])>1.57:	# on doit savoir dans quel sens il est pou savoir si il avance ou recule..?
			sens=FRONT
			print "FRONNNNNNNNNNNNNNNNNNNNNNNNNNNNNNT"
		else:
			sens=BACK

		if len(self.coord) == 1:
				self.tosend = [0,0,self.angle_to_turn(quaternion)]
		elif len(self.coord) == 2:
				self.tosend = [(self.coord[0]-data.pose.pose.position.x)*sens,(self.coord[1]-data.pose.pose.position.y)*sens,0]
		elif len(self.coord) == 3:
				self.tosend = [(self.coord[0]-data.pose.pose.position.x)*sens,(self.coord[1]-data.pose.pose.position.y)*sens,self.angle_to_turn(quaternion)]
		#--- gestion de l inclinaison de la tete 
		(trans,rot) = self.listener.lookupTransform("map", "axis_camera", rospy.Time(0))
		(trans_head_cam,rot_head_cam) = self.listener.lookupTransform("axis_camera", "mon_tf/HeadTouchFront_frame", rospy.Time(0))
		(trans_foot_cam,rot_foot_cam) = self.listener.lookupTransform("/base_link","axis_camera", rospy.Time(0))
		euler_head_cam = euler_from_quaternion(rot_head_cam)
		euler_foot_cam = euler_from_quaternion(rot_foot_cam)
		#yawtest= abs(pi/2-abs(euler_foot_cam[2]))*sign(trans_foot_cam[1])
		yawtosend= atan(trans_foot_cam[1]/trans_foot_cam[0])
		print "euler  ", euler
		print euler_head_cam
		#euler_cam = euler_from_quaternion(rot)
		#print "euler[2]",euler[2]
		#yawtosend = (euler_robot_cam[2]-euler[2])
		#yawtosend = 0
		#print "yawtosend",yawtosend
		if abs(yawtosend)>1.05:
			yawtosend=1.05*sign(yawtosend)
		print "yawtosend post ",yawtosend
		pitchtosend=sign(abs(euler[2])-pi/2)*(-trans[0]+data.pose.pose.position.x)*PITCH_MAX/VISI_MAX*pi/180
		#--- gestion de l inclinaison de la tete  


		# --- on envoie la comande si pas de bug et si eloigne du goal 
		if data.pose.pose.position.z>HAUTEUR_ERREUR :	# to avoid the bug of bad calibration that makes the robot fly 
			print "bug calib or not detected "
			self.move=False
			self.command_head=[yawtosend,pitchtosend]	# TODO

		else:
			self.command_head=[yawtosend,pitchtosend]
			if (abs(self.tosend[0])>FINESSE or abs(self.tosend[1])>FINESSE or abs(self.tosend[2])>FINESSE ):
				self.move=True
				print "True"
				
			else:
				self.move=False
				#self.motionProxy.waitUntilMoveIsFinished()
				print "goal reached"
				rospy.signal_shutdown(' goal reached ')

		print "*****************"
		print "self.command_head" ,self.command_head
		print "tosend" , self.tosend
		print "*****************"
	"""
		
	def timer_callback(self,data):
		if self.detection==1:
			names  = ["HeadYaw", "HeadPitch"]
			angles  = [self.command_head[0], self.command_head[1]]
			fractionMaxSpeed  = HEAD_MAX_SPEED
			#self.motionProxy.setAngles(names, angles, fractionMaxSpeed)  # iciiiiiiiiiii


			print "timer"
			if self.move==True : 
				print "tosend", self.tosend
				#time.wait(3)
				self.motionProxy.post.moveTo(self.tosend[0], self.tosend[1], self.tosend[2])	# sens axes repere pepper = sens axes repere rviz ICIIIIIIIIIIIIIIIIIIIIIIII
				# wait is useful because moveTo is not blocking function
				print "pre wait"
				self.motionProxy.waitUntilMoveIsFinished()
				print "post wait"

		else:
			print "selfdetection=0"

		

		
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

