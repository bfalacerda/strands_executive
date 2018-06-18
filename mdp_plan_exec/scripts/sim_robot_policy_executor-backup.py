#! /usr/bin/env python
import os
import sys
import rospy
import scipy as scipy

from strands_navigation_msgs.msg import NavRoute, ExecutePolicyModeAction, ExecutePolicyModeFeedback, \
    ExecutePolicyModeGoal

from mdp_plan_exec.top_map_mdp import TopMapMdp
from mdp_plan_exec.action_executor import ActionExecutor
from mdp_plan_exec.sim_policy_generator import SimPolicyGenerator

from std_msgs.msg import String
from actionlib import SimpleActionServer, SimpleActionClient
from actionlib_msgs.msg import GoalStatus

from strands_executive_msgs.msg import ExecutePolicyAction, ExecutePolicyFeedback, ExecutePolicyGoal
import numpy as np
from tqdm import tqdm
from scipy.stats import rv_continuous

from rapport.common import *
from rapport.topological_map import *
from rapport.edge_modelling import *
from rapport.stats import *

class SimRobotPolicyExecutor():
    def __init__(self, calc_pol, port, file_dir, file_name):
        self.wait_for_result_dur = rospy.Duration(0.1)
        self.saved_outcomes = {}
        if calc_pol:
            self.current_waypoint_sub = rospy.Subscriber("current_node", String, self.current_waypoint_cb)
            self.closest_waypoint_sub = rospy.Subscriber("closest_node", String, self.closest_waypoint_cb)
            self.mdp = TopMapMdp(explicit_doors=True, forget_doors=True, model_fatal_fails=True)
            self.sim_policy_generator = SimPolicyGenerator(port, file_dir, file_name, self.mdp)
            # self.action_executor = ActionExecutor()
            self.cancelled = False
            self.mdp_as = SimpleActionServer('mdp_plan_exec/execute_policy', ExecutePolicyAction,
                                             execute_cb=self.execute_policy_cb, auto_start=False)
            # self.mdp_as.register_preempt_callback(self.preempt_policy_execution_cb)
            self.mdp_as.start()
        else:
            self.cancelled = False
            self.policy_mdp_file = file_dir
            self.execute_policy()

    def current_waypoint_cb(self, msg):
        self.current_waypoint = msg.data
        self.current_waypoint_sub.unregister()

    def closest_waypoint_cb(self, msg):
        self.closest_waypoint = msg.data
        self.closest_waypoint_sub.unregister()

    def execute_policy_cb(self, goal):
        self.cancelled = False
        self.policy_mdp_file = self.sim_policy_generator.generate_policy_mdp(goal.spec, self.closest_waypoint,
                                                                             rospy.Time.now())
        if self.policy_mdp_file is None:
            rospy.logerr("Failure to build policy for specification: " + goal.spec.ltl_task)
            self.mdp_as.set_aborted()
            return
        self.execute_policy()

    def execute_policy(self):
        self.policy, self.flat_mapping, initial_node = self.load_from_file(self.policy_mdp_file)
        self.current_state = initial_node
        self.current_flat_state = self.flat_mapping['s' + self.current_state]
        durations = []
        successes = 0
        failures = 0
        num_runs = 10000
        for i in tqdm(range(num_runs)):
            total_time = 0
            while self.current_flat_state in self.policy:
                status, duration = self.execute_next_action()
                total_time += duration
                if status != GoalStatus.SUCCEEDED:
                    self.current_flat_state = None
                    break
            if self.cancelled:
                self.cancelled = False
                rospy.loginfo("Policy execution preempted.")
                self.mdp_as.set_preempted()
            elif int(self.current_flat_state) not in self.acc_flat_states:
                failures += 1
            else:
                successes += 1
            # Reset locations to initial values
            self.current_state = initial_node
            self.current_flat_state = self.flat_mapping['s' + self.current_state]
            durations.append(total_time)
        initial_state_details = self.policy['0'][0][2]   # get for the '0'th state (initial). First outcome (as all have the same success probability). 3rd elem is dict.
        print("After {} Simulation runs: \n"
              "|||| Probability of success (sim):  {}\n"
              "|||| Probability of success (MDP):  {}\n"
              "|||| Expected time (sim)         :  {}\n"
              "|||| Expected time (MDP)         :  {}"
              .format(num_runs, successes / float(failures + successes),
                      initial_state_details['prob_succ'],
                      sum(durations) / float(len(durations)),
                      initial_state_details['expected_reward']))
        return (num_runs, durations, successes)

    def execute_next_action(self):
        a, pr, oc = zip(*self.policy[self.current_flat_state])
        # TODO: use models for next state, and then map to closest flat state
        next = np.random.choice(oc, p=pr)
        self.current_flat_state = next['state']
        self.current_state = self.flat_mapping['f' + self.current_flat_state]
        return GoalStatus.SUCCEEDED, float(next['reward'].rvs())  # TODO write a sample method for distributions

    def main(self):
        # Wait for control-c
        rospy.spin()
        if rospy.is_shutdown():
            self.sim_policy_generator.shutdown_prism(True)

    def load_from_file(self, policy_mdp_file):
        policy = {}
        flat_map = {}
        init = None
        skip = True
        tm = self.open_model_builder()
        with open(os.path.join(policy_mdp_file, 'adv.tra'), 'r') as adv_file:
            with open(os.path.join(policy_mdp_file, 'prod.sta'), 'r') as prod_file:
                self.load_policy(adv_file, policy, tm)
                init = self.load_mapping(flat_map, init, prod_file)
        self.set_init_and_acc_states(os.path.join(policy_mdp_file, 'prod.lab'))
        self.load_guarantees(policy, policy_mdp_file, 'guarantees1.vect', 'guarantees2.vect', 'guarantees3.vect')
        self.get_overall_exp_reward(policy)
        return policy, flat_map, init

    def load_policy(self, adv_file, policy, tm):
        skip = True
        for line in adv_file:
            if skip:
                skip = False
            else:
                # parts is of format [initial state, successor state, probability of successor, action]
                parts = line.split(' ')
                if parts[0] not in policy:
                    policy[parts[0]] = []
                model = self.load_model(parts, tm)
                policy[parts[0]].append([parts[3], parts[2],
                                         {'state': parts[1], 'reward': model, 'expected_reward': None,
                                          'prob_succ': None, 'exp_prog_reward': None}])

    def load_mapping(self, flat_map, init, prod_file):
        skip = True
        for line in prod_file:
            if skip:
                skip = False
            else:
                part = line.split(':')
                flat_map['s' + part[1]] = part[0]
                flat_map['f' + part[0]] = part[1]
                if init is None:
                    init = part[1]
        return init

    def set_init_and_acc_states(self, labels_file):
        self.acc_flat_states = []
        f = open(labels_file, 'r')
        line = f.readline()
        label_names = line.split(' ')
        for label in label_names:
            label_pair = label.split('=')
            if label_pair[1].strip('\n') == '"target"':
                acc_index = int(label_pair[0])

        for line in f:
            line = line.split(':')
            state_index = int(line[0])
            labels = line[1].split(' ')
            del labels[0]
            for label in labels:
                if int(label) == acc_index:
                    self.acc_flat_states.append(int(state_index))
        f.close()

    def open_model_builder(self):
        from pymongo import MongoClient

        # create a one-way triangle map from some yaml map
        tm = from_strands_yaml('tpl.yaml')

        # build some default models for the edges
        # build_models_for_map(tm)

        # or load from data
        client = MongoClient('localhost')
        df = import_mongo_nav_stats(client, db = 'strands-db', collection = 'nav_stats')
        build_models_for_map(tm, df)
        return tm

    def load_model(self, line, tm):
        # TODO: Load models properly
        model = tm.edges[line[3].strip()].traversal_model.get_continuous_duration_model()
        print(model)
        return model

    def load_guarantees(self, policy, policy_mdp_file, prob_succ_file, exp_prog_rew_file, exp_reward_file):
        with open(os.path.join(policy_mdp_file, prob_succ_file), 'r') as pr_succ:
            with open(os.path.join(policy_mdp_file, exp_prog_rew_file), 'r') as exp_prog:
                with open(os.path.join(policy_mdp_file, exp_reward_file), 'r') as exp_rew:
                    suc_lines = pr_succ.readlines()
                    prog_lines = exp_prog.readlines()
                    rew_lines = exp_rew.readlines()
                    for i in range(len(suc_lines)):
                        si = str(i)
                        try:
                            pol = policy[si]
                            for j in range(len(pol)):
                                p = pol[j]
                                p[2]['prob_succ'] = float(suc_lines[i])
                                p[2]['exp_prog_reward'] = float(prog_lines[i])
                                p[2]['expected_reward'] = float(rew_lines[i])
                                pol[j] = p
                            policy[si] = pol
                        except KeyError:
                            pass  # States which don't exist in the policy.

    def get_overall_exp_reward(self, policy):
        pass


if __name__ == '__main__':
    # To run independently of ROS. `./src/strands_executive/mdp_plan_exec/scripts/sim_robot_policy_executor.py "False" 8088 saved_prism saved_prism/topo_map.mdp`
    # Where folder `saved_prism contains the precomputed .tra, .sta etc files

    filtered_argv = rospy.myargv(argv=sys.argv)

    if len(filtered_argv) == 5:
        calc_pol = (filtered_argv[1] == 'True')
        print(filtered_argv)
        if calc_pol:
            rospy.init_node('robot_mdp_policy_executor')
        port = filtered_argv[2]
        file_dir = filtered_argv[3]
        model_file = filtered_argv[4]

        mdp_executor = SimRobotPolicyExecutor(calc_pol, int(port), file_dir, model_file)
        if calc_pol:
            mdp_executor.main()
    else:
        rospy.logerr("Usage: rosrun mdp_plan_exec robot_mdp_exec.py calc_pol port file_dir model_file")
