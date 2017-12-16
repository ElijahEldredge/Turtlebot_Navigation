#!/usr/bin/env python

# Team 14: Brandon Knox, Elijah Eldredge, Jon Andrews
# RBE 3002, Unified Robotics IV
# Assignment: Final Term Project

import rospy, tf, copy, math
import aStar, bfs
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, PoseStamped, PointStamped
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry, Path
import numpy as np
from std_msgs.msg import String


class TurtleBot:

    wheel = .23 # based on wheel track from https://yujinrobot.github.io/kobuki/doxygen/enAppendixKobukiParameters.html
    radius = .035

    # sets max and min linear and angular velcoities we would ever want the turtlebot to go
    max_linear = .3
    min_linear = .1
    max_angular = .5
    min_angular = .05

    # creates the details for the stop message
    stop_msg = Twist()
    stop_msg.linear.x = 0
    stop_msg.angular.z = 0
	
    def __init__(self):

        """
            This constructor sets up class variables and pubs/subs
        """
        print 'creating Turtlebot'
        self.transform = tf.TransformerROS()

        # initialize commonly used variables
        self.isInit = False
        self.robot = .38
        self.theta = 0
        self.initPos = [0,0.3,0.3]
        self.initOrient = [0,0,0,0]
        self.hopeful = [0,0,0]
        self.pose = Pose()

        # publishers
        self._vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1) # publish messages to make robot move
        self.wp_pub = rospy.Publisher('/waypoints', Path, queue_size=1) # publish waypoints from aStar path

        # subscribers
        rospy.Subscriber('/here', PoseStamped, self.executeAStar, queue_size=1) # handle nav goal events (wasn't used for final project)
        rospy.Subscriber('/theBottom', PointStamped, self.doBFS,  queue_size=1) # turtlebot starts exploring when point is published
        rospy.Subscriber('/move_base/events/bumper', BumperEvent, self.readBumper, queue_size=1) # handle bumper events
        rospy.Subscriber('/odom', Odometry, self.odomCallback) # constantly updating position and orientation of robot
        rospy.Subscriber('/frontiers', Point, self.executeAStar) # executes aStar search when frontier is found

        # transform listener and broadcaster
        self.odom_tfList = tf.TransformListener()
        self.odom_broad = tf.TransformBroadcaster()

        print 'Made Turtlebot'

    # create bfs object and run searching algorithm
    def doBFS(self,msg):
    	print 'Received Point!'
    	# rotate just under 360 degrees
    	self.rotate(3.1)
    	self.rotate(3.1)
    	# stop the Turtlebot
    	self.stopTurtle()
    	# create new breadth-first search object
    	bff = bfs.bfs()
    	rospy.sleep(.25)
    	# use wavefront to search for frontiers
    	bff.ripple(self.pose.position)
    	print 'completed search'

    # doBFS publishes frontier to a topic which node is subscribed to and runs this
    def executeAStar(self, msg):
    	# create new aStar object
        astart = aStar.aStar()
        rospy.sleep(.25)
        # search for best path 
        astart.justDoIt(self.pose.position, msg)
        # get shortest path and waypoints from aStar
        self.path = astart.getPath()
        wayPoints = astart.wayPoints()
        wpPub = Path()
        wpPub.poses = wayPoints
        wpPub.header.frame_id = 'map'
        # publish path to /waypoints topic
        self.wp_pub.publish(wpPub)

        # for each waypoint, run navToPose to get to it
        for pose in wayPoints:
    		self.navToPose(pose)
    	# stop the robot
    	self.stopTurtle()
    	# search for next frontier using bfs from aStar goal (aka current point now)
    	self.doBFS(msg)


    def navToPose(self, goal):
        """
            This is a callback function. It should extract data from goal, drive in a striaght line to reach the goal and 
            then spin to match the goal orientation.
        """
        # waits for transform
        self.odom_tfList.waitForTransform('odom','base_footprint', rospy.Time(0), rospy.Duration(1.0))
        transGoal = self.odom_tfList.transformPose('base_footprint', goal) # transform the nav goal from the global coordinate system to the robot's coordinate system
        self.desiredPose(transGoal)

        # checks to see if the goal is the same position
        if (self.hopeful[0] == self.pose.position.x and self.hopeful[1] == self.pose.position.y):
        	pass

        # if not calculate how much it needs to turn and distance away from next point
        else:
            initialTurn = math.atan2(self.hopeful[1], self.hopeful[0])
            print initialTurn
            self.rotate(initialTurn)
            distance = math.sqrt(self.hopeful[0]**2 + self.hopeful[1]**2)
            self.driveStraight(.5,distance)

    def executeTrajectory(self):
      """
        See lab manual for the dance the robot has to excute
      """
      self.driveStraight(1,.6)
      self.rotate(-math.pi/2)
      self.driveStraight(1,.45)
      self.rotate((3*math.pi)/4)


    # drives straight a certain distance with a given speed
    def driveStraight(self, speed, distance):
        startX = self.pose.position.x
        startY = self.pose.position.y
        print 'X:', startX, 'Y:', startY

        curDistance = 0

        while curDistance < distance:
        	# scaler for ramped velocity
            scaler = 1/(1 + abs(curDistance - distance/2))
            if scaler >= .25:
                self.moveTurtle(speed,0)
            else:
                self.moveTurtle(4*scaler*speed, 0)
            # calcuate current distance away from goal
            curDistance = math.sqrt((self.pose.position.x - startX)**2 + (self.pose.position.y - startY)**2)
            rospy.sleep(.01)
        self.stopTurtle()

    # calculate desired pose relative to robots position
    def desiredPose(self, goal):

        self.hopeful[0] = goal.pose.position.x
        self.hopeful[1] = goal.pose.position.y

        quarter = goal.pose.orientation
        quarter = [quarter.x, quarter.y, quarter.z, quarter.w]
        roll, pitch, yaw = euler_from_quaternion(quarter)
        self.hopeful[2] = yaw
        print self.hopeful[2]
    
    def spinWheels(self, v_left, v_right, time):
        """
           This method should use differential drive kinematics to compute V and omega (linear x = V, angular z = omega).
           It should then create a ??? message type, and publish it to ??? in order to move the robot
        """

        lin_vel = ((v_left + v_right)/2)
        ang_vel = ((v_right - v_left)/self.wheel)
       
        driveStartTime = rospy.Time.now().secs

        print 'Start time', driveStartTime

        while rospy.Time.now().secs - driveStartTime < time and not rospy.is_shutdown():
            self.moveTurtle(lin_vel, ang_vel)
        self.stopTurtle()
        print 'End time', rospy.Time.now().secs

        
    def rotate(self, angle):
        """
            This method should populate a ??? message type and publish it to ??? in order to spin the robot
        """
        # print 'rotating'
        startAngle = self.theta
        angleDeg = (angle*180)/math.pi
        totAngle = startAngle + angleDeg
        if totAngle > 180:
            goalAngle = totAngle - 360
        elif totAngle < -180:
            goalAngle = totAngle + 360
        else:
            goalAngle = totAngle
        print 'My goal angle is', goalAngle
        while ((self.theta < goalAngle - 1.5) or (self.theta > goalAngle + 1.5)):
        	angSpeed = angle/abs(angle)*.2
        	self.moveTurtle(0,angSpeed)
        self.stopTurtle()

    # updates odometry of robot so it hopefully always knows where it is
    def odomCallback(self, msg):
    	# when first initializing, wait for transform
        if not self.isInit:
            try:
                (position, orientation) = self.odom_tfList.lookupTransform('map','base_footprint', rospy.Time(0))

                self.initPos = position
                self.initOrient = orientation

                roll, pitch, yaw = euler_from_quaternion(orientation)

                self.initTheta = math.degrees(yaw)

                self.isInit = True

            except:
                pass
        # for the rest of the time always update
        else:
            (position, orientation) = self.odom_tfList.lookupTransform('map','base_footprint', rospy.Time(0)) 

            self.pose.position.x = position[0]
            self.pose.position.y = position[1]
            self.pose.position.z = position[2]

            self.pose.orientation.x = orientation[0]
            self.pose.orientation.y = orientation[1]
            self.pose.orientation.z = orientation[2]
            self.pose.orientation.w = orientation[3]
            
            roll, pitch, yaw = euler_from_quaternion(orientation)
            self.theta = math.degrees(yaw)

    # used to send a Twist message with the given linear and/or angular velocity
    def moveTurtle(self, linearVelocity, angularVelocity):
        # Send a movement (twist) message.
        move_msg = Twist()
        # if given linear velocity is greater or less than max or min values, set it to those values
        if linearVelocity > self.max_linear:
        	move_msg.linear.x = self.max_linear
        elif linearVelocity < self.min_linear and linearVelocity > 0:
        	move_msg.linear.x = self.min_linear
        else:
        	move_msg.linear.x = linearVelocity

    	# if given angular velocity is greater or less than max or min values, set it to those values
        if angularVelocity > self.max_angular:
        	move_msg.angular.z = self.max_angular
        elif angularVelocity < self.min_angular and angularVelocity > 0:
        	move_msg.angular.z = self.min_angular
        else:
        	move_msg.angular.z = angularVelocity

    	# publish the Twist message
        self._vel_pub.publish(move_msg)
    
    # stops the Turtlebot
    def stopTurtle(self):
        self._vel_pub.publish(self.stop_msg)
        return True

    def readBumper(self, msg):
       """
        callback function that excutes on a BumperEvent
       """
       if (msg.state == 1):
            self.stopTurtle()
            print "Bumper pressed!"
            self.executeTrajectory()
        

if __name__ == '__main__':
    
    rospy.init_node('team14_lab5')
    # create new Turtlebot object
    turtle = TurtleBot()
    
    while not turtle.isInit:
        pass
    # give the turtle a rest after its hard work of being created
    rospy.sleep(rospy.Duration(1, 0))
    
    #make the robot keep doing something...
    rospy.spin()