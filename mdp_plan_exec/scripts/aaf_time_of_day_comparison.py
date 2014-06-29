#! /usr/bin/env python

import sys
import rospy
import rospkg


from strands_executive_msgs.srv import GetExpectedTravelTime, GetExpectedTravelTimeRequest
from strands_executive_msgs.srv import AddMdpModel, AddMdpModelRequest


from mdp_plan_exec.mdp import TopMapMdp, ProductMdp
from mdp_plan_exec.prism_client import PrismClient

from datetime import datetime, timedelta

import numpy as np
import matplotlib.pyplot as plt

    
class MdpPlanner(object):

    def __init__(self):
    
        # Create the main state machine
        self.time_client= rospy.ServiceProxy('/mdp_plan_exec/get_expected_travel_time_to_node', GetExpectedTravelTime)
        self.add_client= rospy.ServiceProxy('/mdp_plan_exec/add_mdp_model', AddMdpModel)
        self.mdp_file='/home/bruno/Desktop/teste.prism'
        self.top_map_mdp=TopMapMdp('aaf_y1_topo')
        self.prism_client=PrismClient(8085, '/home/bruno/tmp/prism/data')

        
        
    def get_expected_travel_time(self, source, target, time_of_day):
        self.top_map_mdp.set_initial_state_from_name(source)
        #self.exp_times_handler.update_current_top_mdp(req.time_of_day, False)
        self.top_map_mdp.write_prism_model(self.mdp_file)
        self.prism_client.update_model(time_of_day,self.mdp_file)
        specification='R{"time"}min=? [ ( F "' + target + '") ]'
        result=self.prism_client.check_model(time_of_day,specification)
        result=float(result)
        return result

    
    
    def main(self):
        
        time_of_day='test'
        
        
        start_waypoint='SafePointLobby'
        end_waypoint='CrossRoads'
        
        start = datetime(2014, 5, 12)
        end = datetime(2014, 5, 16)
        
        
        #n_days=(end-start).days+1
        n_day_slices = 3
        exp_times=[0]*n_day_slices
        days=[0]*n_day_slices
        
        n_weeks=4
        
        fig = plt.figure()
        ax = fig.add_subplot(111)

        index = np.arange(n_day_slices)
        bar_width = 0.15

        opacity = 0.4
        error_config = {'ecolor': '0.3'}
        
        
        
        day_start=6.5
        mid_morning=9
        after_lunch=11.5
        day_end=14
        
        

        date_query= {"_meta.inserted_at": {"$gte": start, "$lte": end}}
        self.top_map_mdp.update_nav_statistics(date_query=date_query, start_hour=day_start, end_hour=mid_morning)
        #top_map_mdp.set_policy('/home/bruno/tmp/prism/all_day/adv.tra')
        self.top_map_mdp.write_prism_model(self.mdp_file)
        self.prism_client.add_model(time_of_day, self.mdp_file)
        d=self.get_expected_travel_time(start_waypoint,end_waypoint,time_of_day)
        rospy.loginfo(d)
        exp_times[0]=d
        days[0]=0

        self.top_map_mdp.update_nav_statistics(date_query=date_query,start_hour=mid_morning, end_hour=after_lunch)
        #top_map_mdp.set_policy('/home/bruno/tmp/prism/all_day/adv.tra')
        self.top_map_mdp.write_prism_model(self.mdp_file)
        self.prism_client.add_model(time_of_day, self.mdp_file)
        d=self.get_expected_travel_time(start_waypoint,end_waypoint,time_of_day)
        rospy.loginfo(d)
        exp_times[1]=d
        days[1]=1
        
        self.top_map_mdp.update_nav_statistics(date_query=date_query,start_hour=after_lunch, end_hour=day_end)
        #top_map_mdp.set_policy('/home/bruno/tmp/prism/all_day/adv.tra')
        self.top_map_mdp.write_prism_model(self.mdp_file)
        self.prism_client.add_model(time_of_day, self.mdp_file)
        d=self.get_expected_travel_time(start_waypoint,end_waypoint,time_of_day)
        rospy.loginfo(d)
        exp_times[2]=d
        days[2]=2

        
        #self.top_map_mdp.update_nav_statistics(date_query=date_query,start_hour=15, end_hour=18)
        ##top_map_mdp.set_policy('/home/bruno/tmp/prism/all_day/adv.tra')
        #self.top_map_mdp.write_prism_model(self.mdp_file)
        #self.prism_client.add_model(time_of_day, self.mdp_file)
        #d=self.get_expected_travel_time(start_waypoint,end_waypoint,time_of_day)
        #rospy.loginfo(d)
        #exp_times[3]=d
        #days[3]=3        
                

        




        rects1 = ax.bar(index, exp_times, bar_width,
                        alpha=opacity,
                        color='b',
                        label='Week 1')

        #rects2 = plt.bar(index + bar_width, means_women, bar_width,
                        #alpha=opacity,
                        #color='r',
                        #yerr=std_women,
                        #error_kw=error_config,
                        #label='Women')
                        
                        
                        

                        
                        
                        
        start = datetime(2014, 5, 19)
        end = datetime(2014, 5, 23)
        
        
        

        date_query= {"_meta.inserted_at": {"$gte": start, "$lte": end}}
        self.top_map_mdp.update_nav_statistics(date_query=date_query,start_hour=day_start, end_hour=mid_morning)
        #top_map_mdp.set_policy('/home/bruno/tmp/prism/all_day/adv.tra')
        self.top_map_mdp.write_prism_model(self.mdp_file)
        self.prism_client.add_model(time_of_day, self.mdp_file)
        d=self.get_expected_travel_time(start_waypoint,end_waypoint,time_of_day)
        rospy.loginfo(d)
        exp_times[0]=d
        days[0]=0

        self.top_map_mdp.update_nav_statistics(date_query=date_query,start_hour=mid_morning, end_hour=after_lunch)
        #top_map_mdp.set_policy('/home/bruno/tmp/prism/all_day/adv.tra')
        self.top_map_mdp.write_prism_model(self.mdp_file)
        self.prism_client.add_model(time_of_day, self.mdp_file)
        d=self.get_expected_travel_time(start_waypoint,end_waypoint,time_of_day)
        rospy.loginfo(d)
        exp_times[1]=d
        days[1]=1
        
        self.top_map_mdp.update_nav_statistics(date_query=date_query,start_hour=after_lunch, end_hour=day_end)
        #top_map_mdp.set_policy('/home/bruno/tmp/prism/all_day/adv.tra')
        self.top_map_mdp.write_prism_model(self.mdp_file)
        self.prism_client.add_model(time_of_day, self.mdp_file)
        d=self.get_expected_travel_time(start_waypoint,end_waypoint,time_of_day)
        rospy.loginfo(d)
        exp_times[2]=d
        days[2]=2

        
        #self.top_map_mdp.update_nav_statistics(date_query=date_query,start_hour=15, end_hour=18)
        ##top_map_mdp.set_policy('/home/bruno/tmp/prism/all_day/adv.tra')
        #self.top_map_mdp.write_prism_model(self.mdp_file)
        #self.prism_client.add_model(time_of_day, self.mdp_file)
        #d=self.get_expected_travel_time(start_waypoint,end_waypoint,time_of_day)
        #rospy.loginfo(d)
        #exp_times[3]=d
        #days[3]=3
        
        




        rects2 = ax.bar(index+bar_width, exp_times, bar_width,
                        alpha=opacity,
                        color='g',
                        label='Week 2')
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        start = datetime(2014, 5, 26)
        end = datetime(2014, 5 , 30)
        
        
        

        date_query= {"_meta.inserted_at": {"$gte": start, "$lte": end}}
        self.top_map_mdp.update_nav_statistics(date_query=date_query,start_hour=day_start, end_hour=mid_morning)
        #top_map_mdp.set_policy('/home/bruno/tmp/prism/all_day/adv.tra')
        self.top_map_mdp.write_prism_model(self.mdp_file)
        self.prism_client.add_model(time_of_day, self.mdp_file)
        d=self.get_expected_travel_time(start_waypoint,end_waypoint,time_of_day)
        rospy.loginfo(d)
        exp_times[0]=d
        days[0]=0

        self.top_map_mdp.update_nav_statistics(date_query=date_query,start_hour=mid_morning, end_hour=after_lunch)
        #top_map_mdp.set_policy('/home/bruno/tmp/prism/all_day/adv.tra')
        self.top_map_mdp.write_prism_model(self.mdp_file)
        self.prism_client.add_model(time_of_day, self.mdp_file)
        d=self.get_expected_travel_time(start_waypoint,end_waypoint,time_of_day)
        rospy.loginfo(d)
        exp_times[1]=d
        days[1]=1
        
        self.top_map_mdp.update_nav_statistics(date_query=date_query,start_hour=after_lunch, end_hour=day_end)
        #top_map_mdp.set_policy('/home/bruno/tmp/prism/all_day/adv.tra')
        self.top_map_mdp.write_prism_model(self.mdp_file)
        self.prism_client.add_model(time_of_day, self.mdp_file)
        d=self.get_expected_travel_time(start_waypoint,end_waypoint,time_of_day)
        rospy.loginfo(d)
        exp_times[2]=d
        days[2]=2

        
        #self.top_map_mdp.update_nav_statistics(date_query=date_query,start_hour=15, end_hour=18)
        ##top_map_mdp.set_policy('/home/bruno/tmp/prism/all_day/adv.tra')
        #self.top_map_mdp.write_prism_model(self.mdp_file)
        #self.prism_client.add_model(time_of_day, self.mdp_file)
        #d=self.get_expected_travel_time(start_waypoint,end_waypoint,time_of_day)
        #rospy.loginfo(d)
        #exp_times[3]=d
        #days[3]=3        
        
        
        
        rects3 = ax.bar(index+2*bar_width, exp_times, bar_width,
                        alpha=opacity,
                        color='r',
                        label='Week 3')
        
        
        
        
        
        
        
        
        
        
        start = datetime(2014, 6, 2)
        end = datetime(2014, 6, 6)
        

        

        date_query= {"_meta.inserted_at": {"$gte": start, "$lte": end}}
        self.top_map_mdp.update_nav_statistics(date_query=date_query,start_hour=day_start, end_hour=mid_morning)
        #top_map_mdp.set_policy('/home/bruno/tmp/prism/all_day/adv.tra')
        self.top_map_mdp.write_prism_model(self.mdp_file)
        self.prism_client.add_model(time_of_day, self.mdp_file)
        d=self.get_expected_travel_time(start_waypoint,end_waypoint,time_of_day)
        rospy.loginfo(d)
        exp_times[0]=d
        days[0]=0

        self.top_map_mdp.update_nav_statistics(date_query=date_query,start_hour=mid_morning, end_hour=after_lunch)
        #top_map_mdp.set_policy('/home/bruno/tmp/prism/all_day/adv.tra')
        self.top_map_mdp.write_prism_model(self.mdp_file)
        self.prism_client.add_model(time_of_day, self.mdp_file)
        d=self.get_expected_travel_time(start_waypoint,end_waypoint,time_of_day)
        rospy.loginfo(d)
        exp_times[1]=d
        days[1]=1
        
        self.top_map_mdp.update_nav_statistics(date_query=date_query,start_hour=after_lunch, end_hour=day_end)
        #top_map_mdp.set_policy('/home/bruno/tmp/prism/all_day/adv.tra')
        self.top_map_mdp.write_prism_model(self.mdp_file)
        self.prism_client.add_model(time_of_day, self.mdp_file)
        d=self.get_expected_travel_time(start_waypoint,end_waypoint,time_of_day)
        rospy.loginfo(d)
        exp_times[2]=d
        days[2]=2

        
        #self.top_map_mdp.update_nav_statistics(date_query=date_query,start_hour=15, end_hour=18)
        ##top_map_mdp.set_policy('/home/bruno/tmp/prism/all_day/adv.tra')
        #self.top_map_mdp.write_prism_model(self.mdp_file)
        #self.prism_client.add_model(time_of_day, self.mdp_file)
        #d=self.get_expected_travel_time(start_waypoint,end_waypoint,time_of_day)
        #rospy.loginfo(d)
        #exp_times[3]=d
        #days[3]=3        
        
        
        
        rect4 = ax.bar(index+3*bar_width, exp_times, bar_width,
                        alpha=opacity,
                        color='y',
                        label='Week 4')
        
        
        
        
        
        
        
        self.prism_client.shutdown(False)                

        plt.xlabel('Days')
        plt.ylabel('Seconds')
        plt.title('Expected times of travel between WayPoints 1 and 10')
       # plt.xticks(index+bar_width*2.5, days,rotation='vertical')
        plt.legend()
        
        ax.autoscale(tight=True)

       # plt.tight_layout()
        plt.show()
        






if __name__ == '__main__':
    rospy.init_node('test_client')
    

        
    mdp_planner =  MdpPlanner()
    mdp_planner.main()
    
    
