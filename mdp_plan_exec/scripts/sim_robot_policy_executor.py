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
        initial_flat_state = 0
        self.current_state = initial_node
        self.current_flat_state = self.flat_mapping['s' + self.current_state]
        starting_exec = True
        durations = []
        successes = 0
        failures = 0
        num_runs = 10000
        for i in tqdm(range(num_runs)):
            total_time = 0
            while self.current_flat_state in self.policy:
                starting_exec = False
                # current_nav_policy = self.policy_utils.generate_current_nav_policy(self.policy_mdp)
                status, duration = self.execute_next_action()
                total_time += duration
                if status != GoalStatus.SUCCEEDED:
                    self.current_flat_state = None
                    break
                    # self.current_flat_state = self.flat_mapping[self.current_state]
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
        print("After {} Simulation runs: \n"
              "|||| Probability of success (sim):  {}\n"
              "|||| Expected time from simulation: {}"
              .format(num_runs, successes / float(failures + successes),
                      sum(durations) / float(len(durations))))

    def execute_next_action(self):
        a, pr, oc = zip(*self.policy[self.current_flat_state])
        # TODO: use models for next state, and then map to closest flat state
        next = np.random.choice(oc, p=pr)
        self.current_flat_state = next['state']
        self.current_state = self.flat_mapping['f' + self.current_flat_state]
        return GoalStatus.SUCCEEDED, next['reward'].random_state.normal(10, 1)  # TODO write a sample method for distributions

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
        default_time_model = gaussian_gen()
        with open(os.path.join(policy_mdp_file, 'adv.tra'), 'r') as adv_file:
            with open(os.path.join(policy_mdp_file, 'prod.sta'), 'r') as prod_file:
                for line in adv_file:
                    if skip:
                        skip = False
                    else:
                        parts = line.split(' ')
                        if parts[0] not in policy:
                            policy[parts[0]] = []
                        model = self.load_model(parts)
                        time_model = default_time_model if model is None else model
                        policy[parts[0]].append((parts[3], parts[2], {'state': parts[1], 'reward': time_model}))
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
        self.set_init_and_acc_states(os.path.join(policy_mdp_file, 'prod.lab'))
        return policy, flat_map, init

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

    def load_model(self, line):
        # TODO: Load models properly
        return None


class gaussian_gen(rv_continuous):
    "Gaussian distribution"

    def _pdf(self, x):
        return np.exp(-x ** 2 / 2.) / np.sqrt(2.0 * np.pi)


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
