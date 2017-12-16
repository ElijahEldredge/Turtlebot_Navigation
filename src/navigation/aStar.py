#!/usr/bin/env python

# Team 14: Brandon Knox, Elijah Eldredge, Jon Andrews
# RBE 3002, Unified Robotics IV
# Assignment: Final Term Project

import rospy, tf, numpy, math,time
import Node
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry, Path, OccupancyGrid, MapMetaData, GridCells
from geometry_msgs.msg import Twist, PoseStamped

# Node(object)
# This class instantiates A-Star, which includes node checking
# neighbor iteration, location tracking, motion planning, and
# node search
class aStar(object):
    def __init__(self):


    	# robot size definition
        self.resolution = .1
        self.robot = .38

        # definition of costmap
        self.costmap = OccupancyGrid()
        self.points = [] 

        # map subscribers
        self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.createMap, queue_size=1)
        self.map_cost_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.createCostMap, queue_size=1)

        # path publishers
        self.frontier_pub = rospy.Publisher('frontier', GridCells, queue_size=1)
        self.explored_pub = rospy.Publisher('explored', GridCells, queue_size=1)
        self.shortpath_pub  = rospy.Publisher('path', GridCells, queue_size=1)

    # justDoIt() takes the start and end destinations and executes the processes required to
    # move to the location that is defined by the user
    def justDoIt(self, startCoord, goalCoord):
        self.startCoord = startCoord
        self.startNode = Node.Node(startCoord, None, 0)
        self.frontier = []
        self.frontier.append(self.startNode)
        self.explored = []
        self.goalCoord = goalCoord

        rospy.sleep(2)


        # frontier publisher
        self.pub = GridCells()
        self.pub.header = self.map.header
        self.pub.cell_width = self.map.info.resolution
        self.pub.cell_height = self.map.info.resolution

        # explored publisher
        self.exp = GridCells()
        self.exp.header = self.map.header
        self.exp.cell_width = self.map.info.resolution
        self.exp.cell_height = self.map.info.resolution

        # path publishers
        self.path = GridCells()
        self.path.header = self.map.header
        self.path.cell_width = self.map.info.resolution
        self.path.cell_height = self.map.info.resolution

        # # obstacles publisher
        self.obs = GridCells()
        self.obs.header = self.map.header
        self.obs.cell_width = self.map.info.resolution
        self.obs.cell_height = self.map.info.resolution

		self.shortpath_pub.publish(self.path)
        self.pub.cells.append(self.startCoord)
        self.frontier_pub.publish(self.pub)

        # frontier checking, iterating through neighbors
        while not self.frontier[0].contains(goalCoord, self.resolution):
            next = self.frontier.pop(0)
            self.pub.cells.remove(next.coord)
            if self.wasHere(next.coord):
                continue
            self.explored.append(next.coord)
            self.exp.cells.append(next.coord)

            neighbors = next.getNeighbors(self.resolution)

            # checking for neighbors and iterating through them
            for node in neighbors:
                print 'searching for astar neighbor'
                if self.isValid(node.coord):
                    if not self.wasHere(node.coord) and not self.wasFound(node.coord):
                        if (len(self.frontier) == 0):
                            self.frontier.append(node)
                        else:
                            node_cost = node.dist + self.hCost(node.coord, self.goalCoord) #self.goal?
                            print self.hCost(node.coord, self.goalCoord)
                            for i, j in enumerate(self.frontier):
                                j_cost = j.dist + self.hCost(j.coord, self.goalCoord)
                                if ((node_cost - .15 < j_cost) and ((node_cost - node.dist) < (j_cost - j.dist))):
                                    self.frontier.insert(i, node)
                                    break
                                elif (i is len(self.frontier)-1):
                                    self.frontier.append(node)
                                    break

                        self.pub.cells.append(node.coord)
        
            # more publishers
            self.frontier_pub.publish(self.pub)
            self.explored_pub.publish(self.exp)
            self.obstacles_pub.publish(self.obs)
        self.frontier_pub.publish(self.pub)
        self.explored_pub.publish(self.exp)
        time.sleep(.5)
        

    def isValid(self, pos):
        room = self.robot + self.resolution/2
        for x in numpy.arange(pos.x - room, pos.x + room, self.map.info.resolution):
            for y in numpy.arange(pos.y - room, pos.y + room, self.map.info.resolution):
                if x > self.map.info.origin.position.x and y > self.map.info.origin.position.y:

                    xPoint = int((x - self.map.info.origin.position.x)/self.map.info.resolution)
                    yPoint = int((y - self.map.info.origin.position.y)/self.map.info.resolution)
                    indx = xPoint + self.map.info.width*yPoint

                    try:
                        if self.map.data[indx] is 100:
                            self.obs.cells.append(pos)
                            return False
                    except:
                        return False
        return True

    # wasHere() checks to see if has been here before
    def wasHere(self, pos):
        for ex in self.explored:
            if pos.x > (ex.x - self.resolution) and pos.x < (ex.x + self.resolution) and pos.y > (ex.y - self.resolution) and pos.y < (ex.y + self.resolution):
                return True
        return False

    # wasFound() checks for if a new frontier is found
    def wasFound(self, pos):
        for fro in self.frontier:
            if pos.x >= fro.coord.x and pos.x < (fro.coord.x + self.resolution) and pos.y >= fro.coord.y and pos.y < (fro.coord.y + self.resolution):
                return True
        return False

    # hCost() does the heuristic math for A-Star
    def hCost(self, pos, goal):
        return math.sqrt((pos.x - goal.x)**2 + (pos.y - goal.y)**2 + (pos.z - goal.z)**2)

    # createMap() creates the map
    def createMap(self, msg):
        self.map = msg

    # createCostMap() creates the cost map
    def createCostMap(self, msg):
    	self.costmap = msg

    # getPath() gets the path to the frontier
    def getPath(self):
        cell = self.frontier[0]
        while cell is not self.startNode:

            time.sleep(.1)
            self.path.cells.insert(0, cell.coord)
            cell = cell.parent
            self.shortpath_pub.publish(self.path)
        time.sleep(.1)
        self.path.cells.insert(0, self.startCoord)
        self.shortpath_pub.publish(self.path)

            self.path.cells.insert(0, cell.coord)
            cell = cell.parent
            self.shortpath_pub.publish(self.path)

        time.sleep(.1)

        print self.startCoord.x, self.startCoord.y
        self.path.cells.insert(0, self.startCoord)
        self.shortpath_pub.publish(self.path)
        return self.path.cells

    # wayPoints() sets checkpoints along the path, iterating
    # through cells to find the turning position at each point
    # in order to travel to the destination
    def wayPoints(self):
    	self.points = []
        for idx, cell in enumerate(self.path.cells):
            if idx < (len(self.path.cells) - 1):
                nextCell = self.path.cells[idx + 1]
                newTheta = math.degrees(math.atan2((nextCell.y - cell.y),(nextCell.x - cell.x)))
                
                if len(self.points) == 0:
                    wpPose = self.createPose(cell, newTheta)
                    self.points.append(wpPose)
                    curTheta = newTheta
                elif abs(newTheta - curTheta) > .5:
                    wpPose = self.createPose(cell, newTheta)
                    self.points.append(wpPose)
                    curTheta = newTheta

            else:
                newTheta = 0
                wpPose = self.createPose(cell, newTheta)
                self.points.append(wpPose)
                lastPoint = self.points[len(self.points) - 2]
                distFor = math.sqrt((cell.x - lastPoint.pose.position.x)**2 + (cell.y - lastPoint.pose.position.y)**2)

        return self.points

    # createPose() generates the pose for the robot to be in
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