#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from grsim_ros_bridge_msgs.msg import *
from krssg_ssl_msgs.msg import *
import math

ball = Pose()
robot0 = SSL_DetectionRobot()
robot1 = SSL_DetectionRobot()
robot2 = SSL_DetectionRobot()
robot3 = SSL_DetectionRobot()
robot4 = SSL_DetectionRobot()

#inicial
dist0=(0,0)
dist1= (0,0)
dist2= (0,0) 
dist3= (0,0)
dist4= (0,0)
p_ball = (0,0)


def position_ball(data):
	global ball		
	ball = data.balls

def save_ball():

	global p_ball

	try: 
		p_ball=((ball[0].x),(ball[0].y))
		dist(p_ball[0], p_ball[1])
		#print('return')
		return(p_ball)

	except: 
		#print('except') 
		pass

	
def dist(x, y):

	global dist0, dist1

	dist0=(x-robot0.x,y-robot0.y)
	dist1=(x-robot1.x, y-robot1.y)
	dist2=(x-robot2.x, y-robot2.y)
	dist3=(x-robot3.x, y-robot3.y)
	dist4=(x-robot4.x, y-robot4.y)



def jugador_blue(data):
	global robot0,robot1,robot2,robot3,robot4
				
	for i in range(0, len(data.robots_blue)):
		id_robots = data.robots_blue[i].robot_id
		if id_robots == 0:
			robot0 = data.robots_blue[i]
		if id_robots == 1:
			robot1 = data.robots_blue[i]
		if id_robots == 2:
			robot2 = data.robots_blue[i]
		if id_robots == 3:
			robot3 = data.robots_blue[i]
		if id_robots == 4:
			robot4 = data.robots_blue[i]
			
if __name__=="__main__":
	rospy.init_node("detect", anonymous=False)
	
	rospy.Subscriber("/vision", SSL_DetectionFrame, position_ball)
	rospy.Subscriber("/vision", SSL_DetectionFrame, jugador_blue)
	
	r = rospy.Rate(100)
	#goaly= 500
	#goalx= 500

	
	while not rospy.is_shutdown():
   		#goal_angle = math.atan2(goaly - robot0.y, goalx - robot0.x)
		save_ball()
		print(p_ball,'------',dist0)
		

