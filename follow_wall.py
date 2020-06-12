#!/usr/bin/env python

# ROS stuff
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
# other useful math tools
from tf.transformations import euler_from_quaternion
from math import atan2, pow, sqrt

 
# constants
angle_eps = 0.2
dis_eps = 0.01

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

pub = None

# Class that will be used to read and parse /odom topic
class odomReader:

    def __init__(self):

        # subscribing to "/odom" topic
        # function newOdom() will take care of the new incoming message  
        sub = rospy.Subscriber("/odom", Odometry, self.newOdom)
        self.x = None
        self.y = None
        self.theta = None

    # Function that will take care of input message from odom topic
    # This function will be called whenever new message is available to read
    # Subsequently odom topic is parsed to get (x,y,theta) coordinates 
    def newOdom(self, msg):
        # get x and y coordinates
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # convert quaternion to Euler angles
        rot_q = msg.pose.pose.orientation
        (self.roll, self.pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

# Class that is responsible to read and parse raw LaserScan data 
class scanReader:

    def __init__(self):
        # subscribing to "/odom" topic
        # function newOdom() will take care of the new incoming message  
        sub = rospy.Subscriber("/scan", LaserScan, self.newScan)
        # divide laser scan data into 5 regions
        self.region = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }
    # Function that will take care of input message from scan topic
    # This function will be called whenever new message is available to read
    # Subsequently scan topic is parsed to get region data: minimum distance to object from every sight 
    def newScan(self, msg):
        self.ranges = msg.ranges
        self.msg = msg
        self.region['left'] = min(self.ranges[60:100])
        self.region['fleft'] = min(self.ranges[20:60])
        self.region['front'] = min(self.ranges[0:20]+self.ranges[-20:])
        self.region['fright'] = min(self.ranges[300:340])
        self.region['right'] = min(self.ranges[260:300])
        


# divide robot motion in 3 scenario
state_dict = {
    0: 'go to point',
    1: 'wall detected',
    2: 'follow the wall',
}

# define initial scenario
state = 0
dis = 0.7		#the constant distance I use to decide when to perfrom a action
found = False		#tells if the wall has been found by the front sensor


def make_decision(scan):	
    global state, dis, found
    if (found):	#Dependin on what sensor is readin the wall do a action. Onl walk along the wall when fleft and left senros are reading distance below otherwise state = 1
	if(scan.region['front'] > dis and scan.region['fleft'] > dis and scan.region['fright'] > dis and scan.region['right'] > dis and scan.region['left'] > dis):
	#only fleft     
	    state = 1 
	elif(scan.region['front'] < dis and scan.region['fleft'] < dis and scan.region['fright'] > dis and scan.region['right'] > dis and scan.region['left'] < dis):
	#front, fleft, left - range
	    state = 1 
	elif(scan.region['front'] > dis and scan.region['fleft'] < dis and scan.region['fright'] > dis and scan.region['right'] > dis and scan.region['left'] < dis):
	#just fleft, left - range
	    state = 2
	elif(scan.region['front'] < dis and scan.region['fleft'] < dis and scan.region['fright'] > dis and scan.region['right'] > dis and scan.region['left'] > dis):
	#just fleft, front - range
	    state = 1
	elif (scan.region['front'] < dis and scan.region['fleft'] < dis and scan.region['fright'] < dis and scan.region['right'] > dis and scan.region['left'] > dis):
        #wall is fright, front, fleft - in range
            state = 1
	elif(scan.region['front'] > dis and scan.region['fleft'] > dis and scan.region['fright'] > dis and scan.region['right'] > dis and scan.region['left'] > dis):
	#   front, left, right, fleft, fright     - Out of range
	    state = 1
        elif(scan.region['front'] < dis and scan.region['fleft'] < dis and scan.region['fright'] < dis and scan.region['right'] > dis and scan.region['left'] < dis):
	#   front, left, fleft, fright - in range
	    state = 1
    else: #if the wall is not found keep state = 0 (so bot keeps following the goal point)
       state = 0


def main():
    global pub, found, dis	#global variables

    # initialize ROS node
    rospy.init_node("follow_wall")
    # run stop function when this node is killed
    rospy.on_shutdown(stop)
    rospy.sleep(0.5)

    # define the control velocity publisher of topic type Twist
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

    # initialize odom and scan objects
    # Use these objects to access robot position in space and scan information
    odom = odomReader()
    scan = scanReader()
    rospy.sleep(0.5)

    # initialize speed as Twist topic 
    speed = Twist()

    # set the loop frequency
    rate = rospy.Rate(20)

    # Set the goal point 
    goal = Point()
    goal.x = 0.0
    goal.y = 5.0
    

    while not rospy.is_shutdown():
        
        # initialize speed as Twist topic
        speed = Twist()
	make_decision(scan)

        # TODO:
        # Decide what to do for the robot in each of these states:
	if state == 0:
            # go to point state. 

            '''
            This part of the code will make the bot move towards the goal point until the wall is found.
            '''
            inc_x = goal.x - odom.x
	    inc_y = goal.y - odom.y
	    #find the angle between the current position and the goal
	    angle_to_goal = atan2(inc_y, inc_x)
        
            # find the heading angle difference
            angle_diff = angle_to_goal - odom.theta
            if(scan.region['front'] < dis):
                found = True	
	    if(abs(angle_diff) > 0.1):	#keep turning until the angle difference is > 0.1
		speed.linear.x = 0.0
		speed.angular.z = 0.3
	    else:				#if angle difference is right move along x direction
		speed.angular.z = 0.0
		speed.linear.x = 0.2
            
	    print "current state: ", state_dict[state]


        elif state == 1:
            # wall detected state. 
            '''
            Depending on what sensors are being read I will decide wheather to turn right or left. 
	    In this project we mostly turn left except in few situations

            '''
	    
	    #decide on which way to turn
	    if(scan.region['front'] > dis and scan.region['fleft'] > dis and scan.region['fright'] > dis and scan.region['right'] > dis and scan.region['left'] > dis):    
		#turn right = if all the sensors are reading distance aboce dis(variable)
	        speed.linear.x = 0.2
	        speed.angular.z = 0.5
	    elif(scan.region['front'] < dis and scan.region['fleft'] < dis and scan.region['fright'] < dis and scan.region['right'] > dis and scan.region['left'] > dis):
		#front, fleft, fright  - Initially when bot get closer to wall it then turns right though this if statement    
	        speed.linear.x = 0.0
	        speed.angular.z = -0.5
            else:
		speed.angular.z = -0.4         


	    
            print "current state: ", state_dict[state]


        elif state==2:
            # wall following state. 
            '''
            This condition will only apply when the robot is int he right orientation to move straight forward
	    so this section of the code just moves the bot forward in one direction
            '''
	    speed.linear.x = 0.3
	    speed.angular.z = 0.0
            print "current state: ", state_dict[state]


        print scan.region
        pub.publish(speed)
        rate.sleep()

# call this function when you press CTRL+C to stop the robot
def stop():
    global pub
    speed = Twist()
    speed.linear.x = 0.0
    speed.angular.z = 0.0

    pub.publish(speed)

if __name__ == '__main__':
    main()
