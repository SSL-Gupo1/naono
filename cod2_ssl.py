#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import *
from gazebo_msgs.msg import *
from grsim_ros_bridge_msgs.msg import *
from krssg_ssl_msgs.msg import *
import math
import time

pelota = Pose()
robot0 = SSL_DetectionRobot()
robot1 = SSL_DetectionRobot()
robot2 = SSL_DetectionRobot()
robot3 = SSL_DetectionRobot()
robot4 = SSL_DetectionRobot()
robot5 = SSL_DetectionRobot()
robot6 = SSL_DetectionRobot()
robot7 = SSL_DetectionRobot()
robot8 = SSL_DetectionRobot()
robot9 = SSL_DetectionRobot()

#inicial
dist0=(0,0)
dist1= (0,0)
dist2= (0,0) 
dist3= (0,0)
dist4= (0,0)
p_ball = (0,0)
x = 0
y = 0

def position_ball(data):
	global pelota		
	pelota = data.balls
	
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


def save_ball():

	global p_ball

	try: 
		p_ball=((pelota[0].x),(pelota[0].y))
		x = p_ball[0]
		y = p_ball[1]
		print('bola',x, y )
		return p_ball

	except:
		
		print('except') 
		return(0,0)

def goalkeeper(max_x, max_y, min_x, min_y, target_orientation, robot_orientation, pos_robot_x, pos_robot_y, publisher_name):
    velocity = 0.1
    global flag_robot_9
    flag_robot_9 = False
    msg = SSL()
    if (robot_orientation > target_orientation+0.2):
        msg.cmd_vel.angular.z = -0.05
        msg.cmd_vel.linear.x = 0
        msg.cmd_vel.linear.y = 0
    elif (robot_orientation < target_orientation-0.2):
        msg.cmd_vel.angular.z = 0.05
        msg.cmd_vel.linear.x = 0
        msg.cmd_vel.linear.y = 0
    else:
        if(pos_robot_x > max_x):
            msg.cmd_vel.angular.z = 0
            msg.cmd_vel.linear.x = -0.1
            msg.cmd_vel.linear.y = 0
        elif(pos_robot_x < min_x):
            msg.cmd_vel.angular.z = 0
            msg.cmd_vel.linear.x = 0.1
            msg.cmd_vel.linear.y = 0

        if(pos_robot_y > max_y):
            msg.cmd_vel.angular.z = 0
            msg.cmd_vel.linear.x = 0
            msg.cmd_vel.linear.y = -0.3
            flag_robot_9 = True
        elif(pos_robot_y < min_y):
            msg.cmd_vel.angular.z = 0
            msg.cmd_vel.linear.x = 0
            msg.cmd_vel.linear.y = 0.3
            flag_robot_9 = False
        else:
            if(flag_robot_9):
                msg.cmd_vel.angular.z = 0
                msg.cmd_vel.linear.x = 0
                msg.cmd_vel.linear.y = -0.3
            else:
                msg.cmd_vel.angular.z = 0
                msg.cmd_vel.linear.x = 0
                msg.cmd_vel.linear.y = 0.3

    publisher_name.publish(msg)

def shootout(publisher_name):
    msg = SSL()
    time.sleep(5)
    msg.cmd_vel.linear.x = 0.5
    publisher_name.publish(msg)
    time.sleep(1)
    msg.cmd_vel.linear.x = 0.5
    publisher_name.publish(msg)
    msg.kicker = True
    time.sleep(2)
    msg.cmd_vel.linear.x = 0.5
    msg.cmd_vel.angular.z = 0.025
    publisher_name.publish(msg)
    time.sleep(2.5)
    msg.cmd_vel.linear.x = 0.0
    msg.cmd_vel.angular.z = 0.0
    msg.kicker = False
    publisher_name.publish(msg)
    time.sleep(1)
    
def shoot(publisher_name):
    msg = SSL()
    msg.kicker = True
    time.sleep(0.5)
    msg.cmd_vel.linear.x = 0.5
    msg.cmd_vel.angular.z = 0.025
    publisher_name.publish(msg)
    time.sleep(1)
    msg.cmd_vel.linear.x = 0.0
    msg.cmd_vel.angular.z = 0.0
    msg.kicker = False
    publisher_name.publish(msg)
    time.sleep(1)


def dist():

	global dist0, dist1, dist2, dist3, dist4

	soma_0 = (((robot0.x - x) * (robot0.x - x)) + ((robot0.y - y) * (robot0.y - y)))
	dist0 = math.sqrt(soma_0)
	soma_1 = (((robot1.x - x) * (robot1.x - x)) + ((robot1.y - y) * (robot1.y - y)))
	dist1 = math.sqrt(soma_1)
	soma_2 = (((robot0.x - x) * (robot0.x - x)) + ((robot0.y - y) * (robot0.y - y)))
	dist2 = math.sqrt(soma_2)
	soma_3 = (((robot0.x - x) * (robot0.x - x)) + ((robot0.y - y) * (robot0.y - y)))
	dist3 = math.sqrt(soma_3)
	cerca = find_min(dist0,dist1,dist2,dist3)
	if (cerca == dist0):
		return 0
	if (cerca == dist1):
		return 1
	if (cerca == dist2):
		return 2
	if (cerca == dist3):
		return 3

def find_min(num1, num2, num3, num4):
    min_num = num1
    if num2 < min_num:
        min_num = num2
    if num3 < min_num:
        min_num = num3
    if num4 < min_num:
        min_num = num4
    return min_num
def shootout(publisher_name):
    msg = SSL()
    time.sleep(5)
    msg.cmd_vel.linear.x = 0.5
    publisher_name.publish(msg)
    time.sleep(1)
    msg.cmd_vel.linear.x = 0.5
    publisher_name.publish(msg)
    msg.kicker = True
    time.sleep(2)
    msg.cmd_vel.linear.x = 0.5
    msg.cmd_vel.angular.z = 0.025
    publisher_name.publish(msg)
    time.sleep(2.5)
    msg.cmd_vel.linear.x = 0.0
    msg.cmd_vel.angular.z = 0.0
    msg.kicker = False
    publisher_name.publish(msg)
    time.sleep(1)
    
def go_to_target(target_x, target_y, pos_robot_x, pos_robot_y, orientation_robot, vel_linear_x, vel_linear_y, vel_ang_z, heading_thres, distance_thres, publisher_name):
    current_distance = round(math.hypot(target_x - pos_robot_x, target_y - pos_robot_y),2)
    goal_angle = math.atan2(target_y - pos_robot_y, target_x - pos_robot_x)
    heading = goal_angle - orientation_robot

    if heading > math.pi: heading -= 2 * math.pi
    elif heading < -math.pi: heading += 2 * math.pi

    msg = SSL()

    print(heading, current_distance)

    if (heading > heading_thres):
        msg.cmd_vel.angular.z = vel_ang_z
        if (current_distance < 1.0): msg.cmd_vel.linear.x = 0
        else: msg.cmd_vel.linear.x = vel_linear_x/(1+abs(heading*5))
    elif (heading < -heading_thres):
        msg.cmd_vel.angular.z = -vel_ang_z
        if (current_distance < 1.0): msg.cmd_vel.linear.x = 0
        else: msg.cmd_vel.linear.x = vel_linear_x/(1+abs(heading*5))
    else:
        if (current_distance > distance_thres):
            msg.cmd_vel.linear.x = vel_linear_x
            msg.cmd_vel.angular.z = 0.0
    if (current_distance < distance_thres):

        if (orientation_robot < -1.5 or orientation_robot > -0.5):
            msg.cmd_vel.angular.z = 0
            msg.cmd_vel.linear.x = 0.0
            
        else:
            msg.cmd_vel.angular.z = 0.0
            msg.cmd_vel.linear.x = 0.0
            shoot(pub4)
            

    publisher_name.publish(msg)
    print(current_distance)
			
if _name=="main_":
	rospy.init_node("braian_node", anonymous=False)
	
	rospy.Subscriber("/vision", SSL_DetectionFrame, position_ball)
	rospy.Subscriber("/vision", SSL_DetectionFrame, jugador_blue)
	pub = rospy.Publisher('/robot_blue_4/cmd', SSL, queue_size=10)
	pub1 = rospy.Publisher('/robot_blue_0/cmd', SSL, queue_size=10)
	pub3 = rospy.Publisher('/robot_blue_1/cmd', SSL, queue_size=10)
	pub4 = rospy.Publisher('/robot_blue_3/cmd', SSL, queue_size=10)
	
	r = rospy.Rate(10)
	goaly= 500
	goalx= 500
	ssl_msg = SSL()
	bot3blue = SSL()
	xball = 0
	yball = 0
	ssl_msg.cmd_vel.linear.y = -0.3
	while not rospy.is_shutdown():
		pball = save_ball()
		if(pball != (0,0)):
			
			xball = pball[0]
			yball = pball[1]
		print(dist())

		go_to_target(xball,yball,robot3.x,robot3.y,robot3.orientation,0.2,0.2,2,0.3,50,pub4)
        

		print("bola corde  " ,xball,yball)

	

		print("orientation", robot3.orientation)
		

		




		r.sleep()