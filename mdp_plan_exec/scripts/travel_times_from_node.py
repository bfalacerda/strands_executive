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

from strands_perception_people_msgs.msg import PedestrianLocations


    
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

        self.nav_stats=self.read_nav_stats()
        
        self.top_nodes=self.read_top_map()
        

        
        
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
        
        
        
    
    def count_people(self,nav_stat):
        
    

        date=nav_stat[1]["inserted_at"]
        
        nav_time=nav_stat[0].operation_time
        
        
        #date_query= {"_meta.inserted_at": {"$gte": date-timedelta(seconds=nav_stat[0].operation_time-5), "$lte": date + timedelta(seconds=5)}}
        
        start_date=date-timedelta(seconds=nav_stat[0].operation_time)
        end_date=date
        
        
            
        total_detects=0
        detected_ids = []
        
        i=0
        for entry in self.pedestrians:
            entry_date=entry["_meta"]["inserted_at"].replace(tzinfo=date.tzinfo)
            print 'start_date', start_date
            print 'entry', entry["_meta"]["inserted_at"]
            print 'entry', entry_date
            print 'end', end_date
            if entry_date > end_date:
                break
            if entry_date > start_date:
                detects=entry["ids"]
                for person_id in detects:
                    if person_id not in detected_ids:
                        total_detects=total_detects+1
                        detected_ids.append(person_id)
        print total_detects
        
                
    
        return total_detects
        
        
    def find_waypoint_entry(self, waypoint):
        for entry in self.top_nodes:
            if entry[0].name == waypoint:
                return entry[0]
        print "Waypoint not found."
        return None
        
        
        
    def get_distance(self, source, target):
        for entry in self.top_nodes:
            if entry[0].name == source:
                pos_source=[entry[0].pose.position.x, entry[0].pose.position.y]
            if entry[0].name == target:
                pos_target=[entry[0].pose.position.x, entry[0].pose.position.y]
                
        return math.sqrt((pos_source[0] - pos_target[0])**2 + (pos_source[1] - pos_target[1])**2)
    
    
    def get_mean_travel_times(self, source_name, target_name):
        travel_times = []
        for entry in self.nav_stats:
            if  entry[0].origin == source_name and entry[0].target==target_name:
            #if  entry[0].origin == source_name and entry[0].target==target_name    and entry[0].status=='success':
                travel_times.append(entry[0].operation_time)
                
        print np.std(travel_times)/np.mean(travel_times)
       # print sp.rv_continuous.entropy(travel_times)
        if len(travel_times) > 5:
            return np.mean(travel_times), np.std(travel_times)
        else:
            return None, None
       
    
    def evaluate_time_based_estimates(self,get_best_q, n_result, vector):
        np_vector=np.array(vector)
        ordered_indices=np.argsort(np_vector)
        if get_best_q:
            return ordered_indices[:n_result]
        else:
            return ordered_indices[-n_result:]
            
    
    
    def waypoint_stats(self):
        

        #waypoint_entry=self.find_waypoint_entry(waypoint, top_nodes)
        
        sources=[]
        targets=[]
        edge_names=[]
        dist_based_times=[]
        real_expected_times=[]
        std_devs=[]
        relative_dist_based_errors=[]
        #rel_std_devs=[]
        #entropies=[]
        
        
        n_edges=0
        for top_node in self.top_nodes:
            for edge in top_node[0].edges:
                source_name=top_node[0].name
                target_name=edge.node                
                mean, std_dev = self.get_mean_travel_times(source_name, target_name)
                if mean is not None:
                    sources.append(source_name)
                    targets.append(target_name)
                    edge_names.append(source_name + '\n(' + edge.action[0:15] + ')\n' + target_name)  
                    dist=self.get_distance(source_name, edge.node)
                    if edge.action == 'move_base' or edge.action == 'human_aware_navigation':
                        dist_based_time = dist/0.55
                    else:
                        dist_based_time = dist/0.15    
                    dist_based_times.append(dist_based_time)

                    real_expected_times.append(mean)
                    std_devs.append(std_dev)
                    relative_dist_based_errors.append(math.fabs(dist_based_time-mean)/mean)
                    n_edges=n_edges+1
        
        n_results=10
        #sorted_values=self.evaluate_time_based_estimates(True, n_results, relative_dist_based_errors)
        sorted_values=self.evaluate_time_based_estimates(False, n_results, relative_dist_based_errors)
        
        
        sources_aux=[0]*n_results
        targets_aux=[0]*n_results
        edge_names_aux=[0]*n_results
        dist_based_times_aux=[0]*n_results
        real_expected_times_aux=[0]*n_results
        std_devs_aux=[0]*n_results
        relative_dist_based_errors_aux=[0]*n_results
        for i in range(0,n_results):
            sources_aux[i]=sources[sorted_values[i]]
            targets_aux[i]=targets[sorted_values[i]]
            edge_names_aux[i]=edge_names[sorted_values[i]]
            dist_based_times_aux[i]=dist_based_times[sorted_values[i]]
            real_expected_times_aux[i]=real_expected_times[sorted_values[i]]
            std_devs_aux[i]=std_devs[sorted_values[i]]
            relative_dist_based_errors_aux[i]=relative_dist_based_errors[sorted_values[i]]
            
            
            
        sources=sources_aux
        targets=targets_aux
        edge_names=edge_names_aux
        dist_based_times=dist_based_times_aux
        real_expected_times=real_expected_times_aux
        std_devs=std_devs_aux
        relative_dist_based_errors=relative_dist_based_errors_aux 
            
        #for entry in nav_stats:
            #if entry[0].origin == waypoint and entry[0].status=='success':
                #stat_vectors[entry[0].target].append(float(entry[0].operation_time))
                ##people_count[entry[0].target]+=self.count_people(entry)
        
        #data_totals=[]
        #avgs=[]
        #std_devs=[]
       ## people_count_list=[]
        #waypoint_names=[]
        #dist_times_list=[]
        
        #n_targets=0
        #for  key, entry in stat_vectors.iteritems():
            #n_data=len(entry)
            #if n_data > 1:
                #data_totals.append(n_data)
                #waypoint_names.append(key)
                #avgs.append(np.mean(entry))
                #std_devs.append(np.std(entry))
                #n_targets=n_targets+1
                ##people_count_list.append(people_count[key])
                #dist_times_list.append(dist_based_times[key])
                
        
            
            


        fig, ax = plt.subplots()

        index = np.arange(n_results)
        bar_width = 0.2

        opacity = 0.4
        error_config = {'ecolor': '0.3'}

        rects1 = plt.bar(index, real_expected_times, bar_width,
                        alpha=opacity,
                        color='b',
                        yerr=std_devs,
                        error_kw=error_config,
                        label="Expected Times from Data")
                        
        #rects1 = plt.bar(index, [(float(std_dev) / float(avg))*100 for std_dev,avg in zip(sn, avgs)], bar_width,
                        #alpha=opacity,
                        #color='b',
                        #label="Relative Std")
                        
        rects2 = plt.bar(index + bar_width, dist_based_times, bar_width,
                        alpha=opacity,
                        color='r',
                        label="Expected Times from Straight Line Distance")
        
        #rects3 = plt.bar(index + 2*bar_width, [(float(count) / float(total))*100 for count,total in zip(people_count_list, data_totals)], bar_width,
                        #alpha=opacity,
                        #color='g',
                        #label="Avg People Detected per Transversal")                

        plt.xlabel('Edge')
        plt.ylabel('Average Travel Time')
        #plt.title('Mean and stds from ' + self.waypoint)
        plt.xticks(index + bar_width, edge_names)#, rotation='vertical')
        plt.legend()

        plt.tight_layout()
        plt.show()
            
    
    def main(self):
        
        
       
        self.waypoint_stats()





if __name__ == '__main__':
    rospy.init_node('test_client')
    

        
    mdp_planner =  MdpPlanner()
    mdp_planner.main()
    
    
