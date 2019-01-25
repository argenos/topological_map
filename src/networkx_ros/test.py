#!/usr/bin/env python2

import networkx as nx
import rospy


class TopologicalMap(nx.DiGraph):
    def __init__(self, topological_server='~topological_area'):
        self.area_server = rospy.Service(topological_server, Area, self.get_area)
        self.map_server = rospy.Service('~topological_map', TopoMap, self.get_map)
        self.get_pose = rospy.Service('~topological_pose', Pose, self.get.pose)
        self.path_server = rospy.Service('topological_path', TopoPath, self.get_path)

        rospy.init_node('topological_map', anonymous=True)
        rospy.loginfo('Initialized topological map server')

    def add_L1_area(self):
        '''
        Rooms

        :return:
        '''
        pass

    def add_L2_area(self):
        '''
        Locations

        :return:
        '''
        pass

    def add_L3_area(self):
        '''
        Poses

        :return:
        '''
        pass

    def add_area(self, name, level):
        """
        Adding L1, L2, or L3 areas

        :param level:
        :return:
        """

    def add_path(self):
        pass

    def export(self, G):
        data = nx.node_link_data(G)
        del data['graph']
        del data['multigraph']

        for node in data['nodes']:
            del node['id']

        return data
