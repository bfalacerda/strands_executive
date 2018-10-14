#! /usr/bin/env python
import os
import sys
import scipy as scipy
from ast import literal_eval
import numpy as np
from tqdm import tqdm
from scipy.stats import rv_continuous

from mdp_plan_exec.convert_to_mdp import convert_to_mdp_time_as_reward, convert_to_time_mdp
from mdp_plan_exec.sim_policy_generator import SimPolicyGenerator

from rapport.common import *
from rapport.topological_map import *
from rapport.edge_modelling import *
from rapport.stats import *

import pymongo

class SimulatorNoRos():
    def __init__(self, port, file_dir, file_name, gen_new_pol, use_time):
        self.saved_outcomes = {}
        self.waypoint_map = self.load_wp_map(file_dir, file_name)
        self.generate_new_policy = gen_new_pol
        self.use_time = use_time
        self.cancelled = False
        self.policy_mdp_file = file_dir

    def execute_policy(self, initial_wp, goal_wp, time_limit):
        self.policy, self.flat_mapping, initial_node = self.load_from_file(self.policy_mdp_file, initial_wp, goal_wp, time_limit)
        self.current_state = initial_node
        self.current_flat_state = self.flat_mapping['s',  self.current_state]
        durations = []
        final_state = []
        successes = 0
        failures = 0
        num_runs = 10000
        for i in tqdm(range(num_runs)):
            total_time = 0
            while self.current_flat_state in self.policy:
                success, duration = self.execute_next_action(total_time)
                total_time += duration
                if not success:
                    self.current_flat_state = None
                    break
            if self.current_flat_state is None:
                failures += 1
            elif int(self.current_flat_state) not in self.acc_flat_states and total_time > time_limit:
                failures += 1
            else:
                successes += 1
            if self.use_time:
                final_state.append(self.current_state)
            # Reset locations to initial values
            self.current_state = initial_node
            self.current_flat_state = self.flat_mapping['s', self.current_state]
            durations.append(total_time)
        initial_state_details = self.guarantees[self.flat_mapping['s', initial_node]]
        print("After {} Simulation runs: \n"
              "|||| Probability of success (sim)            :   {}\n"
              "|||| Probability of success (MDP)            :   {}\n"
              "|||| Average total duration                  :   {}\n"
              "|||| Expected time (MDP)                     :   {}\n"
              "|||| Average end state duration of MDP       :   {}"
              .format(num_runs, successes / float(failures + successes),
                      initial_state_details['prob_succ'],
                      sum(durations) / float(len(durations)),
                      initial_state_details['expected_reward'] if not self.use_time else "",
                      sum([time for (a, w, time) in final_state]) / max(1, float(len(final_state)))
                      ))
        return (num_runs, durations, successes, final_state, self.waypoint_map[goal_wp])

    def execute_next_action(self, current_time):
        a, oc = zip(*self.policy[self.current_flat_state])
        next = np.random.choice(a).strip()
        model = self.load_model(next, self.tm)
        # Assume we either succesfully traverse the edge, or fail, can't end up in a different waypoint
        next_state = self.tm.edges[next].end.name
        duration = list(model.sample())
        duration[1] = max(duration[1], 0)
        if not self.use_time:
            # Find state with matching waypoint index in order to find the state of the automata
            for outcome in oc:
                if self.flat_mapping['f', outcome][1] == self.waypoint_map[next_state]:
                    next_aut_state = self.flat_mapping['f', outcome][0]
                    break
            if next_aut_state is None:
                return GoalStatus.ABORTED, duration[1]
            self.current_state = (next_aut_state, self.waypoint_map[next_state])
        else:
            matching = []
            # Find all states with matching waypoint
            for outcome in oc:
                if self.flat_mapping['f', outcome][1] == self.waypoint_map[next_state]:
                    matching.append(outcome)
            best_diff = float("inf")
            best = None
            # Find closest matching duration
            for match in matching:
                state = self.flat_mapping['f', match]
                # TODO: decide whether to use curent sum of sampled times, or just previously mapped to mdp state to get the current time
                # if abs(state[2] - (literal_eval(self.current_state)[2]+duration[1])) < best_diff:
                    # best_diff = abs(state[2] - (literal_eval(self.current_state)[2]+duration[1]))
                if abs(state[2] - (current_time+duration[1])) < best_diff:
                    best_diff = abs(state[2] - (current_time+duration[1]))
                    best = match
            if best is None:
                return False, duration[1]
            self.current_flat_state = best
            self.current_state = self.flat_mapping['f', self.current_flat_state]
        self.current_flat_state = self.flat_mapping['s', self.current_state]
        return ((True if duration[0] == 'success' else False), duration[1])


    def load_from_file(self, policy_mdp_file, initial_wp, goal_wp, time_limit):
        policy = {}
        flat_map = {}
        init = None
        skip = True
        self.tm = self.open_model_builder()
        #TODO: Extract this to be neater
        if self.generate_new_policy:
            if self.use_time:
                convert_to_time_mdp(self.tm, "/home/james/rob_plan_ws/test_mdp.mdp", initial_wp)
                policy_mdp_file = self.sim_policy_generator.generate_policy_mdp('(F "{}" & time < {})'.format(goal_wp, time_limit), True)
            else:
                convert_to_mdp_time_as_reward(self.tm, "/home/james/rob_plan_ws/test_mdp.mdp", initial_wp)
                policy_mdp_file = self.sim_policy_generator.generate_policy_mdp('(F "{}")'.format(goal_wp), False)
        with open(os.path.join(policy_mdp_file, 'adv.tra'), 'r') as adv_file:
            with open(os.path.join(policy_mdp_file, 'prod.sta'), 'r') as prod_file:
                self.load_policy(adv_file, policy, self.tm)
                init = self.load_mapping(flat_map, init, prod_file)
        self.set_init_and_acc_states(os.path.join(policy_mdp_file, 'prod.lab'))
        if self.use_time:
            self.load_guarantees(policy, policy_mdp_file, 'guarantees.vect')
        else:
            self.load_guarantees(policy, policy_mdp_file, 'guarantees1.vect', 'guarantees2.vect', 'guarantees3.vect')
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
                    policy[literal_eval(parts[0])] = []
                policy[literal_eval(parts[0])].append((parts[3], int(parts[1])))

    def load_wp_map(self, directory, mdp_file):
        map = {}
        with open(mdp_file, 'r') as mdp:
            for line in mdp:
                if line.startswith('label'):
                    line = line.replace("label ", "")
                    line = line.split("=")
                    wp_name = line[0].replace("\"", "").strip()
                    wp_index = int(line[2].partition(")")[0])
                    map[wp_name] = wp_index
        return map

    def load_mapping(self, flat_map, init, prod_file):
        skip = True
        for line in prod_file:
            if skip:
                skip = False
            else:
                part = line.split(':')
                flat_map['s', literal_eval(part[1].strip())] = literal_eval(part[0].strip())
                flat_map['f', literal_eval(part[0].strip())] = literal_eval(part[1].strip())
                if init is None:
                    init = literal_eval(part[1].strip())
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
            if label_pair[1].strip('\n') == '"init"':
                init_index = int(label_pair[0])


        for line in f:
            line = line.split(':')
            state_index = int(line[0])
            if(len(line) > 1):
                labels = line[1].split(' ')
            del labels[0]
            for label in labels:
                if int(label) == acc_index:
                    self.acc_flat_states.append(int(state_index))
                if int(label) == init_index:
                    self.current_flat_state = int(state_index)
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

    def load_model(self, action, tm):
        # TODO: Load models properly
        model = tm.edges[action].traversal_model.get_continuous_duration_model()
        return model

    def load_guarantees(self, policy, policy_mdp_file, prob_succ_file, exp_prog_rew_file, exp_reward_file):
        self.guarantees = []
        with open(os.path.join(policy_mdp_file, prob_succ_file), 'r') as pr_succ:
            with open(os.path.join(policy_mdp_file, exp_prog_rew_file), 'r') as exp_prog:
                with open(os.path.join(policy_mdp_file, exp_reward_file), 'r') as exp_rew:
                    suc_lines = pr_succ.readlines()
                    prog_lines = exp_prog.readlines()
                    rew_lines = exp_rew.readlines()
                    for i in range(len(suc_lines)):
                        try:
                            pol = policy[i]
                            self.guarantees.append({'prob_succ': float(suc_lines[i]), 'exp_prog_reward': float(prog_lines[i]), 'expected_reward': float(rew_lines[i])})
                        except KeyError:
                            pass  # States which don't exist in the policy.

    def load_guarantees(self, policy, policy_mdp_file, prob_succ_file):
        self.guarantees = []
        with open(os.path.join(policy_mdp_file, prob_succ_file), 'r') as pr_succ:
            suc_lines = pr_succ.readlines()
            for i in range(len(suc_lines)):
                try:
                    pol = policy[i]
                    self.guarantees.append({'prob_succ': float(suc_lines[i])})
                except KeyError:
                    pass  # States which don't exist in the policy.


if __name__ == '__main__':
    # To run independently of ROS. `./src/strands_executive/mdp_plan_exec/scripts/sim_robot_policy_executor.py "False" 8088 saved_prism saved_prism/topo_map.mdp`
    # Where folder `saved_prism contains the precomputed .tra, .sta etc files
    filtered_argv = rospy.myargv(argv=sys.argv)

    if len(filtered_argv) == 7:
        gen_new_pol = (filtered_argv[1] == 'True')
        use_time = (filtered_argv[2] == 'True')
        port = filtered_argv[3]
        file_dir = filtered_argv[4]
        model_file = filtered_argv[5]

        mdp_executor = SimRobotPolicyExecutor(int(port), file_dir, model_file, gen_new_pol, use_time)
        mdp_executor.execute_policy()

    else:
        rospy.logerr("Usage: rosrun mdp_plan_exec robot_mdp_exec.py use_ros port file_dir model_file")
