from __future__ import print_function

import networkx as nx
from sympy import Polygon, Point

import rospy

import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from topological_map_ros.srv import TopologicalPath, TopologicalPathResponse, TopologicalPosition, TopologicalPositionResponse


def generate_map():
    G = nx.DiGraph()

    G.add_edge('hallway', 'entrance', through='entrance door shown in the map', level='room')
    G.add_edge('hallway', 'bar', through='entrance', level='room')
    G.add_edge('hallway', 'bedroom', through='door', level='room')
    G.add_edge('hallway', 'living room', through='corridor oposite the entrance door', level='room')
    G.add_edge('bedroom', 'kitchen', through='door next to the desk', level='room')
    G.add_edge('kitchen', 'living room', through='corridor between the white drawer and the blue trash can', level='room')
    G.add_edge('living room', 'exit', through='door next to the coat hanger', level='room')

    G.add_edge('entrance', 'hallway', through='entrance door', level='room')
    G.add_edge( 'bar', 'hallway', through='corridor oposite the bar table', level='room')
    G.add_edge('bedroom', 'hallway',  through='door oposite the desk', level='room')
    G.add_edge('living room', 'hallway',  through='corridor between the tv and the bookcase', level='room')
    G.add_edge('kitchen', 'bedroom',  through='door between the cupboard and the blue trash can', level='room')
    G.add_edge('living room', 'kitchen',  through='corridor between the bookcase and the high table', level='room')
    G.add_edge('exit', 'living room',  through='exit door shown in the map', level='room')

    # return G.to_directed()
    return G


class TopologicalMap(object):
    def __init__(self):

        self.server = rospy.Service('topological_path_plan', TopologicalPath, self.handle_path_request)
        self.position_server = rospy.Service('topological_position', TopologicalPosition, self.handle_position_request)

        self.rooms_config = rospy.get_param('~rooms', dict())

        self.rooms = dict()
        # TODO get room poses from param
        for room, points in self.rooms_config.items():
            self.rooms[room] = self.add_room(points)

        self.G = generate_map()

        rospy.loginfo("Initialized topological map server")

    def add_room(self, point_list):
        # TODO use the point list loaded from the config, each point in the list needs to be changed to a tuple like below
        #point_list = [(-0.9909883, -4.218833), (-1.92709, 0.9022037), (-7.009388, -1.916794), (-4.107592, -7.078834)]
        p1, p2, p3, p4 = map(Point, point_list)
        room = Polygon(p1,p2,p3,p4)
        return room

    def handle_path_request(self, req):
        rospy.loginfo("Planning from %s to %s" % (req.source, req.goal))

        reply = TopologicalPathResponse()
        try:
            path = nx.dijkstra_path(self.G, req.source, req.goal)
        except nx.NodeNotFound as e:
            rospy.logerr('failed to find path: ' + e.message)
            return reply

        directions = list()
        start = path.pop(0)
        while path:
            goal = path[0]

            connection = self.G.edges[start, goal]['through']
            instruction = 'Go from the %s to the %s through the %s' % (start, goal, connection)
            rospy.loginfo(instruction)
            directions.append(instruction)

            start = path.pop(0)

        reply.path = path
        reply.directions = directions
        return reply

    def handle_position_request(self, req):
        x, y = req.current_pose.pose.position.x, req.current_pose.pose.position.y
        rospy.loginfo("Received request for topological pose: x={0:.3f}, y={1:.3f} (frame: {2})"
                      .format(x, y, req.current_pose.header.frame_id))
        robot_pose = Point(x, y) # Inspection test pose
        for room in self.rooms:
            if not self.rooms[room].encloses_point(robot_pose):
                continue
            rospy.loginfo('robot is in room: ' + room)
            return TopologicalPositionResponse(room)

        position = 'hallway'
        rospy.logwarn("Failed to find room, using default location: %s" % position )
        return TopologicalPositionResponse(position)
