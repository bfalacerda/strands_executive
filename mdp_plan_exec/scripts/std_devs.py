#! /usr/bin/env python

import sys
import rospy
import rospkg

import math

from ros_datacentre.message_store import MessageStoreProxy
from strands_navigation_msgs.msg import TopologicalNode
from strands_navigation_msgs.msg import NavStatistics

from datetime import datetime, timedelta

import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as sp


from strands_navigation_msgs.msg import NavRoute

    
class MdpPlanner(object):

    def __init__(self):
        #self.top_map='aaf_y1_topo'
        self.top_map='g4s'
        
        self.nav_stats=self.read_nav_stats()
        self.top_nodes=self.read_top_map()  
        
        self.stat_publisher = rospy.Publisher('/top_nodes_std', NavRoute)
    


        
        
    def read_top_map(self):
        msg_store = MessageStoreProxy(collection='topological_maps')
    
        query_meta = {}
        query_meta["pointset"] =self.top_map
        available = len(msg_store.query(TopologicalNode._type, {}, query_meta)) > 0


        if available <= 0 :
            rospy.logerr("Desired pointset '"+point_set+"' not in datacentre")
            rospy.logerr("Available pointsets: "+str(available))
            raise Exception("Can't find waypoints.")
    
        else :
            query_meta = {}
            query_meta["pointset"] = self.top_map
            message_list = msg_store.query(TopologicalNode._type, {}, query_meta)
    
        return message_list
        
        
    def read_nav_stats(self):
        msg_store = MessageStoreProxy(collection='message_store')
    
        query_meta = {}
        query_meta["pointset"] =self.top_map
        available = len(msg_store.query(NavStatistics._type, {}, query_meta)) > 0


        if available <= 0 :
            rospy.logerr("No data")
            raise Exception("Can't find waypoints.")
        else :
            query_meta = {}
            query_meta["pointset"] = self.top_map
            message_list = msg_store.query(NavStatistics._type, {}, query_meta)
    
        return message_list

    
    def get_rel_std_dev(self, source_name, target_name):
        travel_times = []
        for entry in self.nav_stats:
            if  entry[0].origin == source_name and entry[0].target==target_name:
            #if  entry[0].origin == source_name and entry[0].target==target_name    and entry[0].status=='success':
                travel_times.append(entry[0].operation_time)
                
       # print sp.rv_continuous.entropy(travel_times)
        
        return np.std(travel_times)/np.mean(travel_times)#, sp.rv_continuous.entropy(travel_times)
    
    
    def main(self):

        
        sources=[]
        targets=[]
        rel_std_devs=[]
        entropies=[]
        
        for source in self.top_nodes:
            source_name = source[0].name
            for edge in source[0].edges:
                target_name = edge.node
                sources.append(source_name)
                targets.append(target_name)
                std_dev = self.get_rel_std_dev(source_name, target_name)
                rel_std_devs.append(std_dev)
                #entropies.append(entropy)
                
                
        stat_message = NavRoute()
        stat_message.source = sources
        stat_message.target = targets
        stat_message.prob = rel_std_devs
        
        print(max(stat_message.prob))
        for i in range(0,10):
            self.stat_publisher.publish(stat_message)
            rospy.sleep(0.1)
                
        
        






if __name__ == '__main__':
    rospy.init_node('test_client')
    

        
    mdp_planner =  MdpPlanner()
    mdp_planner.main()
    
    
