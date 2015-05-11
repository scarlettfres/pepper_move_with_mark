#!/usr/bin/env python
import roslib
import rospy
import naoqi
import time
import sys
import numpy as np
from visualization_msgs.msg import Marker
from std_msgs.msg import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist # for sending commands to the drone
from naoqi import ALProxy
from tf.transformations import quaternion_from_euler, euler_from_quaternion

FINESSE=0.05
#IP="127.0.0.1"
IP="10.0.206.111"
PORT=9559
class move_pepper:
	def __init__(self,coord,ID):
		
		self.motionProxy = ALProxy("ALMotion",IP,PORT)
		self.postureProxy = ALProxy("ALRobotPosture", IP,PORT)
		self.move=False
		tab_coord=str(coord).split()
		self.coord=[float(tab_coord[0]), float(tab_coord[1]), float(tab_coord[2])]
		print self.coord
		self.clock=rospy.Time.now()



		self.id_markeur_tete=ID
		rospy.Subscriber("/result_position", Marker,self.position_callback)
		rospy.Timer(rospy.Duration(0.5), self.timer_callback)


		
	def position_callback(self,data):
		print "coucou"
		self.clock= rospy.Time.now()
		self.tosend = [self.coord[0]-data.pose.position.x,self.coord[1]-data.pose.position.y]	# repere rviz
		if abs(self.tosend[0])>FINESSE or abs(self.tosend[1])>FINESSE:
			self.move=True
			print "True"
		else:
			self.move=False
			print "false"
			#rospy.signal_shutdown(' goal reached ')


	def timer_callback(self,data):
		print "timer"
		if self.move==True and rospy.Time.now()-self.clock<0.3:
			print "mooove"
			self.motionProxy.post.moveTo(self.tosend[0], self.tosend[1], 0)	# sens axes repere pepper = sens axes repere rviz 
			# wait is useful because with porospy.Time.now()st moveTo is not blocking function
			print "pre wait"
			print "aaaaaaaaaaaaaaaaaaaaaaaaa"
			self.motionProxy.waitUntilMoveIsFinished()
			print "post wait"

		else:
			print " pepper out of range "



	"""
    def angle_callback(self,data):
		if data.id==self.id_markeur_tete:
            self.clock_verif_erreur = rospy.Time.now() + rospy.Duration(temps_erreur)
            self.detection=1
        elif rospy.Time.now()> self.clock_verif_erreur:
            self.detection=0
            print "non detection de la marque ", self.id_markeur_tete

    """
"""
		
	def move(self,x,y,z,xr,yr,zr):
	
		topic = 'visualizationnaoleft'
		
		publisher = rospy.Publisher(topic, Marker)

		markerl = Marker()
		markerl.header.frame_id = "/hydra_base"
		markerl.type = markerl.SPHERE
		markerl.action = markerl.ADD
		markerl.scale.x = 0.2
		markerl.scale.y = 0.2
		markerl.scale.z = 0.2
		markerl.color.a = 1.0
		markerl.pose.orientation.w = 1.0
		markerl.pose.position.x = vectl[0]
		markerl.pose.position.y = vectl[1]
		markerl.pose.position.z = vectl[2]
		markerl.color.r = 0.0
		markerl.color.g = 1.0
		markerl.color.b = 0.0
		markerl.color.a = 1.0
		
		
		
		topic = 'visualizationnaoright'
		publisher2 = rospy.Publisher(topic, Marker)

		markerr = Marker()
		markerr.header.frame_id = "/hydra_base"
		markerr.type = markerr.SPHERE
		markerr.action = markerr.ADD
		markerr.scale.x = 0.2
		markerr.scale.y = 0.2
		markerr.scale.z = 0.2
		markerr.color.a = 1.0
		markerr.pose.orientation.w = 1.0
		markerr.pose.position.x = vectr[0]
		markerr.pose.position.y = vectr[1]
		markerr.pose.position.z = vectr[2]
		markerr.color.r = 0.0
		markerr.color.g = 0.0
		markerr.color.b = 0.1
		markerr.color.a = 1.0
		
		
		self.compteur= self.compteur+1 
		#self.compteur=50
		if self.compteur == 200:
		
			PositionLeft = self.motionProxy.getPosition('LArm',0, True)
			PositionRight = self.motionProxy.getPosition('RArm',0, True)
		
			vectl= [x/RATIO+self.XleftNao,y/RATIO+self.YleftNao,z/RATIO+self.ZleftNao,self.r1leftNao,self.r2leftNao,self.r3leftNao]
			vectr= [xr/RATIO+self.XrightNao,yr/RATIO+self.YrightNao,zr/RATIO+self.ZrightNao,self.r1rightNao,self.r2rightNao,self.r3rightNao]
		
		
			ecartx = abs(PositionLeft[0]-(x/RATIO+self.XleftNao))
			ecarty = abs(PositionLeft[1]-(y/RATIO+self.YleftNao))
			ecartz = abs(PositionLeft[2]-(z/RATIO+self.ZleftNao))
			ecartrx = abs(PositionRight[0]-(xr/RATIO+self.XrightNao))
			ecartry = abs(PositionLeft[1]+(yr/RATIO+self.YrightNao))
			ecartrz = abs(PositionLeft[2]-(zr/RATIO+self.ZrightNao))
		
		
			if (ecartx>FINESSE or ecarty>FINESSE or ecartz>FINESSE):	# on lui envoie une commande que SI mvmt consequent
				
				self.motionProxy.setPosition ("LArm", 0, vectl, VITESSE, 7)
				
			if ecartrx>FINESSE or ecartry>FINESSE or ecartrz>FINESSE:	
				
				self.motionProxy.setPosition ("RArm", 0,vectr, VITESSE, 7)
			self.compteur=0
			time.sleep(0.5)
			publisher.publish(markerl)
			publisher2.publish(markerr)		

	"""
		
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
	'coord': "0 0 0",
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

