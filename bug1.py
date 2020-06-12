#!/usr/bin/env python

# ROS stuff
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
# other useful math tools
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt

 
# global constants
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
        
        #print "range[90]: ", msg.ranges[90]


# divide robot motion in 3 scenario
state_dict = {
    0: 'go to goal',
    1: 'circumnavigate obstacle',
    2: 'change goal',
}
# define initial scenario
state = 0


def get_distance(point1, point2):
    distance = sqrt((point1.x-point2.x)**2 + (point1.y-point2.y)**2)
    return distance

def main():
    global pub, state
    tick = 0			#This is used to create delay after current_goal has been updated

    # initialize ROS node
    rospy.init_node("bug_1")
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
    rate = rospy.Rate(25)

    # Set the goal point 
    goal = Point()
    current_goal = Point()
    goal.x = -3.0
    goal.y = 3.0

    current_goal = goal #goal is out main goal (final destination)
    dis = 0.35

    # arbitrary far away coordinate from goal
    closest_point = Point()
    closest_point.x = 1000
    closest_point.y = 1000

    # arbitrary large number representing inf distance to goal
    closest_dist = 1000 
    # Variable that stores the coordinate of hit point when you 
    # encounter obstacle for the first time
    hit_point = Point()
    temp = 0

    while not rospy.is_shutdown():
        # initialize speed as Twist topic
        speed = Twist()

        # TODO:

        # Decide what to do for the robot in each of these states:
        if state == 0:
            # go to goal state. 
            '''
            Hint: 
                Here robot should go towards a the goal unless it encounters an obstacle. When it encounters the wall
                it should change the state to "circumnavigate obstacle".

                should be bunch of if-else statements

            '''
	    inc_x = current_goal.x - odom.x
	    inc_y = current_goal.y - odom.y
	    #find the angle between the current position and the goal
	    angle_to_goal = atan2(inc_y, inc_x)
        
            # find the heading angle difference
            angle_diff = angle_to_goal - odom.theta

	    #find the distance between goal and current postion
	    dist_diff = sqrt(inc_x**2 + inc_y**2)
	
	    if(scan.region['front'] > dis):
		if(abs(angle_diff) > 0.1):			#keep turning until the angle difference is > 0.1
		    speed.linear.x = 0.0
		    speed.angular.z = 0.2
		elif(current_goal == goal and dist_diff < 0.1 and temp > 0):	#if hit_point is found and goal location is reached change state to terminate
		    state = 2	
	        else:						#if angle difference is right move along x direction
		    speed.angular.z = 0.0
		    speed.linear.x = 0.2
	    else:						#if the wall is found front then start circumnaviate state
		hit_point.x = odom.x				#if the wall is find the update the hit point, state, speed and current goal
		hit_point.y = odom.y
		state = 1
		speed.angular.z = 0.0
		speed.linear.x = 0.0
		current_goal = hit_point
		temp = 1					#this tell me that we have found out hit point
            print "current state: ", state_dict[state]


        elif state == 1:
            # circumnavigate obstacle. 
            '''
            Hint: 
                Here robot should turn right/left based on your choice. And, circumnavigate the obstacle using wall following
                algorithm from previous project. While in this state record closest point to goal where you can head towards goal.
                This state terminates when
                you reach the same point when you hit the obstacle.

                Finally, do not forget to change the state!

                should be bunch of if-else statements

            '''
            # find current distance to goal
	    if(current_goal == hit_point):		#keep updating the closest distance if the current goal is hit point
		dist_to_goal = get_distance(goal, odom)
            	if dist_to_goal<closest_dist:
            		'''Update closest distance and record corresponding coordinate to closest_point variable'''
                	closest_dist = dist_to_goal
			closest_point.x = odom.x
			closest_point.y = odom.y
			
	    if(tick < 250):    				#this is used to create some dealy after the current goal has been updated
		print("This is the number of ticks " + str(tick))
	    	current_distance = 0.3
		tick += 1
	    else:					#if the dealy is over then continue getting the distance from curren goal to current position
		current_distance = get_distance(odom, current_goal)
	    
	    print("This is the current distance " + str(current_distance) + " This is the closest distance to goal " + str(closest_dist))
	
	    
	    #depending on the condition below the bot turn in specific direction - condition statement below prints which sensors meet the distance requirement to wall
	    if(current_distance > 0.15):
	        if (scan.region['left']> dis and scan.region['fleft']>dis and scan.region['front']>dis and scan.region['fright']<dis and scan.region['right']<dis):
		    speed.angular.z = 0.0
		    speed.linear.x = 0.25	#keep moving forward
		    print("Condition 1 - fright, right")
 	        elif (scan.region['left']> dis and scan.region['fleft']>dis and scan.region['front']<dis and scan.region['fright']<dis and scan.region['right']<dis):
 		    speed.angular.z = 0.5	#turn left
		    speed.linear.x = 0.0
		    print("Condition 2 - front, fright, right")
	        elif (scan.region['left']> dis and scan.region['fleft']<dis and scan.region['front']<dis and scan.region['fright']<dis and scan.region['right']>dis):
		    speed.angular.z = 0.5	#turn left
		    speed.linear.x = 0.0
		    print("Condition 3 - fleft, front, fright")
	        elif (scan.region['left']> dis and scan.region['fleft']<dis and scan.region['front']<dis and scan.region['fright']<dis and scan.region['right']<dis):
		    speed.angular.z = 0.9	#turn left
		    speed.linear.x = 0.0
		    print("Condition 4 - fleft, front, right, fright")
	        elif (scan.region['left']> dis and scan.region['fleft']>dis and scan.region['front']>dis and scan.region['fright']>dis and scan.region['right']<dis):
		    speed.angular.z = -0.6	#turn right
		    speed.linear.x = 0.07
		    print("Condition 5 - right")
	        elif (scan.region['left']> dis and scan.region['fleft']>dis and scan.region['front']<dis and scan.region['fright']<dis and scan.region['right']>dis):
		    speed.angular.z = 0.9	#turn left
		    speed.linear.x = 0.0
		    print("Condition 6 - front, fright")
		else:				#if none of the conditions above meet then keep turning left
		    speed.angular.z = 0.7 
		    speed.linear.x = 0.0
            else:		#if current goal is found then change state to get the new goal
		state = 2
		speed.linear.x = 0
		speed.angular.y = 0	    
		tick = 0	
            print "current state: ", state_dict[state]


        elif state == 2:
            # change the goal
            '''
            Hint: 
                Here robot should go back to closest point encountered in state 1. Once you reach that point, change the state
                to go to goal.

                should be bunch of if-else statements
            '''
	    if(current_goal == hit_point):		#if current goal is hitpoitn then set the next goal as closest_point
		current_goal = closest_point
	    	state = 1
		print("My current goal is  - closest_point : " + str(closest_point.x) + ", " + str(closest_point.y))
	    elif(current_goal == closest_point):	#if currentgoal is closest_point then set the next goal as final goal
		current_goal = goal
		state = 0
		current_distance = get_distance(current_goal, odom)
		print("My current goal is  - goal: " + str(current_goal.x) + ", " + str(current_goal.y))
	    else:					#if current goal is goal and this condition is called then terminate the simulation
		print("goal has been reached!!!")
		break

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
