#!/usr/bin/env python
from __future__ import print_function

import networkx as nx
from sympy import Polygon, Point

import rospy

import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

from topological_map.srv import TopologicalPath, TopologicalPathResponse, TopologicalPosition, TopologicalPositionResponse

class TopologicalMap(object):
    def __init__(self):

        self.server = rospy.Service('topological_path_plan', TopologicalPath, handle_path_request)
        self.position_server = rospy.Service('topological_position', TopologicalPosition, handle_position_request)

        self.rooms_config = rospy.get_param('rooms')
        print(self.rooms_config)

        self.rooms = dict()
        # TODO get room poses from param
        for room, points in self.rooms_config.items():
            self.rooms[room] = add_room(points)

        self.G = nx.Graph()


        rospy.init_node('topological_map_server')
        rospy.loginfo("Initialized topological map server")



    def add_room(self, point_list):
        # TODO use the point list loaded from the config, each point in the list needs to be changed to a tuple like below
        point_list = [(-0.9909883, -4.218833), (-1.92709, 0.9022037), (-7.009388, -1.916794), (-4.107592, -7.078834)]
        p1, p2, p3, p4 = map(Point, point_list)
        room = Polygon(p1,p2,p3,p4)
        return room

    def handle_path_request(self, req):
        rospy.loginfo("Planning from %s to %s" % (req.source, req.goal))

        path = nx.dijkstra_path(self.G, req.source, req.goal)

        path = ['hallway', 'bedroom', 'kitchen', 'dishwasher']
        reply = TopologicalPathResponse()
        reply.path = path
        return reply

    def handle_position_request(self, req):
        rospy.loginfo("Received request for topological pose")

        # TODO get this from the request
        x = -1.5
        y = -3.7

        robot_pose = Point(x, y) # Inspection test pose
        # for room in self.rooms:
        room.encloses_point(robot_pose)



        position = 'hallway'
        rospy.loginfo("Robot is in %s" % position )
        return TopologicalPositionResponse(position)
if __name__ == "__main__":
    server = TopologicalMap()
    rospy.spin()
