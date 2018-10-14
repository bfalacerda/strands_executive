#! /usr/bin/env python

from mdp_plan_exec.mdp import Mdp, MdpTransitionDef, MdpPropDef
import rospy
from strands_navigation_msgs.srv import GetTopologicalMap



class HumanNavMdp(Mdp):
    def __init__(self, top_map_name):
        Mdp.__init__(self)
        self.robot_prob = 0.7
        self.human_probs = [0.6, 0.8, 1.0]
        self.human_costs = [1.0, 1.0, 1.0]
        
        self.performance_increase_probs = [0.1, 0.1, 0.1]
        self.performance_decrease_probs = [0.1, 0.1, 0.1]
        self.new_top_map = False
        top_map_srv = rospy.ServiceProxy("/topological_map_publisher/get_topological_map", GetTopologicalMap)
        self.top_map = top_map_srv(top_map_name).map
        print self.top_map

        
        self.create_mdp()




    def create_mdp(self):
        self.n_state_vars=3
        self.state_vars=['waypoint', 'rob_con', 'human_level']
        n_human_levels = len(self.human_probs)
        self.initial_state={'waypoint':0, 'rob_con':1, 'human_level':n_human_levels - 1}
        n_waypoints=len(self.top_map.nodes)
        self.state_vars_range={'waypoint':(-1,n_waypoints-1), 'rob_con':(0,1), 'human_level':(0, n_human_levels - 1)}

        i=0
        for node in self.top_map.nodes:
            waypoint_name=node.name
            self.props.append(waypoint_name)            
            self.props_def[waypoint_name]=MdpPropDef(name=waypoint_name,
                                                    conds={'waypoint':i})
            i=i+1
            
        self.reward_names=['human_cost']
        
        self.n_actions = 0
        self.actions = []
        self.transitions = []
        i = 0
        for node in self.top_map.nodes:
            source_name=node.name
            for edge in node.edges:
                target_index=self.props.index(edge.node)
                action_name=edge.edge_id
                for j in range(0, n_human_levels):
                    trans=MdpTransitionDef(action_name="rob_" + action_name,
                                                    pre_conds={'waypoint':i, "rob_con":1,'human_level':j},
                                                    prob_post_conds=[(self.robot_prob*self.performance_increase_probs[j], {'waypoint':target_index, 'human_level':min(j + 1, n_human_levels - 1)}),
                                                                    (self.robot_prob*(1.0-self.performance_increase_probs[j]), {'waypoint':target_index}),
                                                                    ((1.0-self.robot_prob), {'waypoint':-1})
                                                                     ],
                                                    rewards={'human_cost':0.0})
                                                                    
                    self.transitions.append(trans)
                    trans=MdpTransitionDef(action_name="human_" + action_name,
                                                    pre_conds={'waypoint':i, "rob_con":0,'human_level':j},
                                                    prob_post_conds=[(self.human_probs[j]*self.performance_decrease_probs[j], {'waypoint':target_index, 'human_level':max(j - 1, 0)}),
                                                                    (self.human_probs[j]*(1.0-self.performance_decrease_probs[j]), {'waypoint':target_index}),
                                                                    ((1.0-self.human_probs[j]), {'waypoint':-1})
                                                                     ],
                                                    rewards={'human_cost':1.0})
                                                                    
                    self.transitions.append(trans)
                    
                self.n_actions=self.n_actions + 2
                self.actions.append("rob_" + action_name)
                self.actions.append("human_" + action_name)
            i+=1

        trans=MdpTransitionDef(action_name="give_control",
                                pre_conds={"rob_con":1},
                                prob_post_conds=[(1.0, {"rob_con":0})],
                                rewards={'human_cost':0.0})
        self.transitions.append(trans)
        trans=MdpTransitionDef(action_name="take_control",
                                pre_conds={"rob_con":0},
                                prob_post_conds=[(1.0, {"rob_con":1})],
                                rewards={'human_cost':0.0})
        self.transitions.append(trans)
        self.n_actios = self.n_actions + 2
                

if __name__ == '__main__':
    rospy.init_node('human')
    
    map_name = "tsc_1605"
    mdp = HumanNavMdp(map_name)
    mdp.write_prism_model("/home/bruno/Desktop/human_test.prism")