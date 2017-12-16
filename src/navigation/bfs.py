#!/usr/bin/env python

# Team 14: Brandon Knox, Elijah Eldredge, Jon Andrews
# RBE 3002, Unified Robotics IV
# Assignment: Final Term Project

import rospy, tf, numpy, math,time
import Node
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry, Path, OccupancyGrid, MapMetaData, GridCells
from geometry_msgs.msg import Twist, PoseStamped, Point

# bfs(object)
# This class instantiates breadth-first search in order to
# iterate through the nodes and explore the frontier creates
# a "ripple" in order to explore via wavefront
class bfs(object):
    def __init__(self):

        # robot size definition
        self.resolution = .1
        self.robot = .38

        # map subsribers
        self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.createMap, queue_size=1)
        self.map_cost_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.createCostMap, queue_size=1)
        
        # map publishers
        self.bfs_pub = rospy.Publisher("/move_base/local_costmap/bfs", GridCells, queue_size=1)
        self.fro_pub = rospy.Publisher('/frontiers', Point, queue_size=1)

    # ripple() iterates through nodes to explore the new spaces as a wavefront,
    # inspects each neighbor node and adds it to the list of toGo or was here
    # based on the value
    def ripple(self, startCoord):
        self.startCoord = startCoord
        self.startNode = Node.Node(startCoord, None, 0)
        self.toGo = [self.startNode]
        self.explored = []

        rospy.sleep(3)

        while 1:
            print 'exploring new node'

            try:
                next = self.toGo.pop(0)
            except:
                print 'Finished Exploring!'
                break
            
            if self.wasHere(next.coord):
                continue
            
            self.explored.append(next.coord)
            
            if not self.isKnown(next.coord):
                self.fro_pub.publish(next.coord)
                break
            
            neighbors = next.getNeighbors(self.resolution)
            
            # nested for loop and if statements
            for node in neighbors:
                print 'looking at neighbor'
            
                if self.isValid(node.coord):
                    if len(self.toGo) == 0:
                        self.toGo.append(node)
                    else:
                        node_cost = node.dist
                        for i, j in enumerate(self.toGo):
                            j_cost = j.dist
                            if (node_cost < j_cost):
                                self.toGo.insert(i, node)
                                break
                            elif (i is len(self.toGo)-1):
                                self.toGo.append(node)
                                break
            
                print len(self.toGo)
        
    # isValid() checks to see if the position given is a valid space
    # that the robot could occupy
    def isValid(self, pos):
        room = self.robot + self.resolution/2
        
        for x in numpy.arange(pos.x - room, pos.x + room, self.map.info.resolution):
            for y in numpy.arange(pos.y - room, pos.y + room, self.map.info.resolution):
                if not ((x < self.map.info.origin.position.x) and (y < self.map.info.origin.position.y)):
                    
                    xPoint = int((x - self.map.info.origin.position.x)/self.map.info.resolution)
                    yPoint = int((y - self.map.info.origin.position.y)/self.map.info.resolution)
                    indx = xPoint + self.map.info.width*yPoint
                    
                    # if it is valid, give it a value of closer to 100
                    try:
                        if self.map.data[indx] is 100:
                            return False
                    except:
                        return False
        
        return True

    # isKnown checks to see the node is known, and if so to assign it a value of -1
    def isKnown(self,pos):
        room = self.robot + self.resolution/2
        
        for x in numpy.arange(pos.x - room, pos.x + room, self.map.info.resolution):
            for y in numpy.arange(pos.y - room, pos.y + room, self.map.info.resolution):
                if not ((x < self.map.info.origin.position.x) and (y < self.map.info.origin.position.y)):
        
                    xPoint = int((x - self.map.info.origin.position.x)/self.map.info.resolution)
                    yPoint = int((y - self.map.info.origin.position.y)/self.map.info.resolution)
                    indx = xPoint + self.map.info.width*yPoint
                    
                    # if is known, assign it a value of -1
                    try:
                        if self.map.data[indx] is -1:
                            return False
                    except:
                        return False
        
        return True

    # neighborsNotKnow() checks to see if the neighbor is known, if not, returns false
    def neighborsNotKnown(self, nodes):
        for node in nodes:
        
            if self.isKnown(node.coord):
                return False
                break
        
        return True

    # wasHere() checks to see if the robot has been here before by checking the components
    # of the previous pose
    def wasHere(self, pos):
        for ex in self.explored:
            if pos.x > (ex.x - self.resolution) and pos.x < (ex.x + self.resolution) and pos.y > (ex.y - self.resolution) and pos.y < (ex.y + self.resolution):
                return True
        return False

    # wasFound() checks to see if the node has been previously found in the frontier
    def wasFound(self, pos):
        for fro in self.toGo:
            if pos.x >= fro.coord.x and pos.x < (fro.coord.x + self.resolution) and pos.y >= fro.coord.y and pos.y < (fro.coord.y + self.resolution):
                return True
        return False

    # createMap() creates the map from msg
    def createMap(self, msg):
        print 'map created'
        self.map = msg

    # createCostMap() creates the costmap from msg
    def createCostMap(self, msg):
    	self.costmap = msg

    # createPose() creates the final pose for the robot to end up
    def createPose(self, pos, theta):
        tempPose = PoseStamped()
        tempPose.header.frame_id = 'map'
        tempPose.pose.position.x = pos.x
        tempPose.pose.position.y = pos.y
        tempPose.pose.position.z = 0
        orientation = quaternion_from_euler(0, 0, (theta*math.pi/180))
        tempPose.pose.orientation.x = orientation[0]
        tempPose.pose.orientation.y = orientation[1]
        tempPose.pose.orientation.z = orientation[2]
        tempPose.pose.orientation.w = orientation[3]
        return tempPose