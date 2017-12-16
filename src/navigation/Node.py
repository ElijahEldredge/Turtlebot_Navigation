#!/usr/bin/env python

# Team 14: Brandon Knox, Elijah Eldredge, Jon Andrews
# RBE 3002, Unified Robotics IV
# Assignment: Final Term Project

import math, geometry_msgs, time
from geometry_msgs.msg import Point

# Node(object)
# This class generates node objects, giving them aspects of 
# coordinates, parents, and distance. Instansiating both node
# neighbors and node contents
class Node(object):
	# definitions:
	def __init__(self, coord, parent, dist):
		self.coord = coord
		self.parent = parent
		self.dist = dist

	# getNeighbors returns node neighbors given a resolution, 
	# creating a new list and appending each new neighbor to it
	def getNeighbors(self, res):
		myNeighbors =[]
		for x in [-res,0,res]:
			for y in [-res,0,res]:
				time.sleep(.001)
				pos = Point()
				pos.x = self.coord.x + x
				pos.y = self.coord.y + y
				pos.z = self.coord.z

				newNode = Node(pos, self, self.dist + math.sqrt(x**2 + y**2))

				# if statement to add only new nodes
				if self.contains(pos, res):
					pass
				else:
					myNeighbors.append(newNode)

		return myNeighbors

	# contains checks if the res/2 is less than or greater to the
	# individual contents of the node in the x,y,z or parent
	def contains(self, pos, res):
		if self.coord.x + res/2 > pos.x and self.coord.x - res/2 < pos.x and self.coord.y + res/2 > pos.y and self.coord.y - res/2 < pos.y:
			return True
		elif not self.parent:
			return False
		else:
			return self.parent.contains(pos, res)