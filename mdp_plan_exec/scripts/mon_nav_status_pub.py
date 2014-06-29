#! /usr/bin/env python

import sys
import rospy
import rospkg
import pymongo
import math


from strands_executive_msgs.srv import GetExpectedTravelTime, GetExpectedTravelTimeRequest
from strands_executive_msgs.srv import AddMdpModel, AddMdpModelRequest


from mdp_plan_exec.mdp import TopMapMdp, ProductMdp
from mdp_plan_exec.prism_client import PrismClient

from datetime import datetime, timedelta

import numpy as np
import matplotlib.pyplot as plt


from ros_datacentre.message_store import MessageStoreProxy
from strands_navigation_msgs.msg import TopologicalNode
from strands_navigation_msgs.msg import NavStatistics
from strands_navigation_msgs.msg import MonitoredNavEvent

from strands_perception_people_msgs.msg import PedestrianLocations
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

    
class MdpPlanner(object):

    def __init__(self):
        host=rospy.get_param("datacentre_host")
        port=rospy.get_param("datacentre_port")
        client=pymongo.MongoClient(host,port)
        


        #self.top_map='g4s'
        self.top_map='aaf_y1_topo'
        
        
        self.waypoint='WayPoint1'
        #self.waypoint='CrossRoads'
        
        self.pedestrians = client.message_store.people_tracks.find({"_meta.stored_type":"strands_perception_people_msgs/PedestrianLocations"})
        
        
        self.bumper_not_helped_pub=rospy.Publisher('/bumper_not_helped', PoseArray)        
        self.bumper_helped_pub=rospy.Publisher('/bumper_helped', PoseArray)
        self.nav_not_helped_pub=rospy.Publisher('/nav_not_helped', PoseArray)
        self.nav_helped_pub=rospy.Publisher('/nav_helped', PoseArray)
        
        self.marker_pub=rospy.Publisher('/mon_nav_stats', MarkerArray)
        self.marker_id = 0
        

    def create_marker(self, pose, color, marker_type):
        marker = Marker()
        marker.header.frame_id = "/map"
            #marker.header.stamp = rospy.now()
            #marker.type = marker.ARROW
           
        marker.type = marker_type
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        if color == 'light_green':
            marker.color.r = 0.0/255.0 
            marker.color.g = 80.0/255.0
            marker.color.b = 0.0/255.0
        elif color == 'light_red':
            marker.color.r = 120.0/255.0 
            marker.color.g = 0.0/255.0
            marker.color.b = 0.0/255.0
        elif color == 'dark_green':
            marker.color.r = 0.0/255.0 
            marker.color.g = 255.0/255.0
            marker.color.b = 0.0/255.0
        elif color == 'dark_red':
            marker.color.r = 255.0/255.0 
            marker.color.g = 0.0/255.0
            marker.color.b = 0.0/255.0
        marker.pose = pose
        marker.id = self.marker_id
        self.marker_id=self.marker_id+1
        return marker

        
    def read_mon_nav_stats(self):    
        msg_store = MessageStoreProxy(collection='monitored_nav_events')
    
        query_meta = {}
        available = len(msg_store.query(MonitoredNavEvent._type, {}, query_meta)) > 0


        if available <= 0 :
            rospy.logerr("No data")
            raise Exception("Can't find waypoints.")
        else :
            query_meta = {}
            message_list = msg_store.query(MonitoredNavEvent._type, {}, query_meta)
        
        header=Header()
        header.frame_id='/map'
        bumper_helped_poses=PoseArray(header=header)
        bumper_not_helped_poses=PoseArray(header=header)
        nav_helped_poses=PoseArray(header=header)
        nav_not_helped_poses=PoseArray(header=header)
        
        markers=MarkerArray()
        
        for entry in message_list:
            if entry[0].recover_mechanism == 'bumper_recovery':
                if entry[0].was_helped == True:
                    bumper_helped_poses.poses.append(entry[0].event_end_pose)
                    markers.markers.append(self.create_marker(entry[0].event_end_pose,'dark_green', Marker.SPHERE))
                else:
                    bumper_not_helped_poses.poses.append(entry[0].event_end_pose)
                    markers.markers.append(self.create_marker(entry[0].event_end_pose,'dark_red', Marker.SPHERE))
            elif entry[0].recover_mechanism == 'nav_help_recovery':
                if entry[0].was_helped == True:
                    nav_helped_poses.poses.append(entry[0].event_end_pose)
                    markers.markers.append(self.create_marker(entry[0].event_end_pose,'light_green', Marker.CUBE))
                else:
                    nav_not_helped_poses.poses.append(entry[0].event_end_pose)
                    markers.markers.append(self.create_marker(entry[0].event_end_pose,'light_red', Marker.CUBE))
          
        
             
            
        self.bumper_not_helped_pub.publish(bumper_not_helped_poses)      
        self.bumper_helped_pub.publish(bumper_helped_poses)  
        self.nav_not_helped_pub.publish(nav_not_helped_poses)  
        self.nav_helped_pub.publish(nav_helped_poses) 
        
        self.marker_pub.publish(markers)
        self.marker_id=0

            
    
    def main(self):
        
        
       
        self.read_mon_nav_stats()





if __name__ == '__main__':
    rospy.init_node('test_client')
    

        
    mdp_planner =  MdpPlanner()
    mdp_planner.main()
    
    
