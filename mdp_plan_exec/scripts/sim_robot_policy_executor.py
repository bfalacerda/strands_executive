#! /usr/bin/env python

import sys
import rospy

from strands_navigation_msgs.msg import NavRoute, ExecutePolicyModeAction, ExecutePolicyModeFeedback, \
    ExecutePolicyModeGoal

from mdp_plan_exec.top_map_mdp import TopMapMdp
from mdp_plan_exec.action_executor import ActionExecutor
from mdp_plan_exec.policy_execution_utils import PolicyExecutionUtils

from std_msgs.msg import String
from actionlib import SimpleActionServer, SimpleActionClient
from actionlib_msgs.msg import GoalStatus

from strands_executive_msgs.msg import ExecutePolicyAction, ExecutePolicyFeedback, ExecutePolicyGoal
import numpy as np
from tqdm import tqdm

class SimRobotPolicyExecutor():
    def __init__(self, port, file_dir, file_name):
        self.wait_for_result_dur = rospy.Duration(0.1)
        self.current_waypoint_sub = rospy.Subscriber("current_node", String, self.current_waypoint_cb)
        self.closest_waypoint_sub = rospy.Subscriber("closest_node", String, self.closest_waypoint_cb)
        self.mdp = TopMapMdp(explicit_doors=True, forget_doors=True, model_fatal_fails=True)
        self.policy_utils = PolicyExecutionUtils(port, file_dir, file_name, self.mdp)
        # self.action_executor = ActionExecutor()
        self.saved_outcomes = {}
        self.cancelled = False
        self.mdp_as = SimpleActionServer('mdp_plan_exec/execute_policy', ExecutePolicyAction,
                                         execute_cb=self.execute_policy_cb, auto_start=False)
        # self.mdp_as.register_preempt_callback(self.preempt_policy_execution_cb)
        self.mdp_as.start()

    def current_waypoint_cb(self, msg):
        self.current_waypoint = msg.data
        self.current_waypoint_sub.unregister()

    def closest_waypoint_cb(self, msg):
        self.closest_waypoint = msg.data
        self.closest_waypoint_sub.unregister()

    def execute_nav_policy(self, nav_policy_msg):
        status = GoalStatus.ABORTED
        time = 0
        while self.current_waypoint in nav_policy_msg.source:
            n = nav_policy_msg.source.index(self.current_waypoint)
            edge = nav_policy_msg.edge_id[n]
            target_node = edge.split('_')[1]

            action = self.policy_mdp.get_current_action()
            # filter list of transitions for the transitions for the current action
            if not self.saved_outcomes.has_key(action):
                self.saved_outcomes[action] = [transitions for transitions in self.mdp.transitions if transitions.action_name == action][0]
            # sample the distribution of outcomes for the next state
            p, o = zip(*self.saved_outcomes[action].prob_post_conds)
            next_mdp_state = np.random.choice(o, p=p)['waypoint']
            try:
                next_waypoint = [wp for wp in self.mdp.props_def if self.mdp.props_def[wp].conds['waypoint'] == next_mdp_state][0]
            except Exception:
                status = GoalStatus.ABORTED
                return status, time+self.saved_outcomes[action].rewards['time']
            # TODO: Instead of using expected time from MDP, sample a model
            # edge_duration = edge_model.sample(edge)
            time += self.saved_outcomes[action].rewards['time']

            self.current_waypoint = next_waypoint
            self.closest_waypoint = next_waypoint
            next_state = self.policy_utils.get_next_nav_policy_state(self.current_waypoint, self.policy_mdp)
            self.policy_mdp.set_current_state(next_state)
            # print(next_waypoint)
            if target_node == next_waypoint:
                status = GoalStatus.SUCCEEDED
            else:
                status = GoalStatus.ABORTED
                return status, time
        return status, time

    def execute_policy_cb(self, goal):
        self.cancelled = False
        self.policy_mdp = self.policy_utils.generate_policy_mdp(goal.spec, self.closest_waypoint, rospy.Time.now())
        initial_node = self.closest_waypoint
        initial_flat_state = self.policy_mdp.current_flat_state
        if self.policy_mdp is None:
            rospy.logerr("Failure to build policy for specification: " + goal.spec.ltl_task)
            self.mdp_as.set_aborted()
            return

        # self.publish_feedback(None, None, self.policy_mdp.get_current_action())
        starting_exec = True  # used to make sure the robot executes calls topological navigation at least once before executing non-nav actions. This is too ensure the robot navigates to the exact pose of a waypoint before executing an action there
        durations = []
        successes = 0
        failures = 0
        num_runs = 10000
        for i in tqdm(range(num_runs)):
            total_time = 0
            while (self.policy_mdp.has_action_defined() and not self.cancelled) or starting_exec:
                next_action = self.policy_mdp.get_current_action()
                if next_action in self.mdp.nav_actions or starting_exec:
                    starting_exec = False
                    current_nav_policy = self.policy_utils.generate_current_nav_policy(self.policy_mdp)
                    status, duration = self.execute_nav_policy(current_nav_policy)
                    total_time += duration
                    # rospy.loginfo(
                    #     "Topological navigation execute policy action server exited with status: " + GoalStatus.to_string(
                    #         status))
                    if status != GoalStatus.SUCCEEDED:
                        self.policy_mdp.set_current_state(None)
                        break

                else:
                    print(next_action)
                #     (status, state_update) = self.action_executor.execute_action(self.mdp.action_descriptions[next_action])
                #     executed_action = next_action
                #     print(executed_action)
                #     if not self.cancelled:
                #         next_flat_state = self.policy_utils.get_next_state_from_action_outcome(state_update,
                #                                                                                self.policy_mdp)
                #         self.policy_mdp.set_current_state(next_flat_state)
                #         if next_flat_state is None:
                #             rospy.logerr("Error finding next state after action execution. Aborting...")
                #             break
                #         next_action = self.policy_mdp.get_current_action()
                #         self.publish_feedback(executed_action, status, next_action)
            if self.cancelled:
                self.cancelled = False
                rospy.loginfo("Policy execution preempted.")
                self.mdp_as.set_preempted()
            elif self.policy_mdp.current_flat_state is None:
                # rospy.loginfo("Policy execution failed.")
                # self.mdp_as.set_aborted()
                failures += 1
            else:
                # rospy.loginfo("Policy execution successful.")
                # self.mdp_as.set_succeeded()
                successes += 1
            # Reset locations to initial values
            self.current_waypoint = initial_node
            self.closest_waypoint = initial_node
            self.policy_mdp.set_current_state(initial_flat_state)
            durations.append(total_time)
        print("After {} Simulation runs: \n"
              "|||| Probability of success (sim):  {}\n"
              "|||| Probability of success (MDP):  {}\n"
              "|||| Expected time from simulation: {}\n"
              "|||| Expected time from MDP:        {}"
              .format(num_runs, successes / float(failures + successes),
                      self.policy_mdp.get_guarantees_at_flat_state(initial_flat_state)[0],
                      sum(durations)/float(len(durations)),
                      self.policy_mdp.get_guarantees_at_flat_state(initial_flat_state)[2].to_sec()))

    # def preempt_policy_execution_cb(self):
    #     self.top_nav_policy_exec.cancel_all_goals()
    #     self.action_executor.cancel_all_goals()
    #     self.cancelled = True

    def main(self):
        # Wait for control-c
        rospy.spin()
        if rospy.is_shutdown():
            self.policy_utils.shutdown_prism(True)


if __name__ == '__main__':
    rospy.init_node('robot_mdp_policy_executor')

    filtered_argv = rospy.myargv(argv=sys.argv)

    if len(filtered_argv) != 4:
        rospy.logerr("Usage: rosrun mdp_plan_exec robot_mdp_exec.py port file_dir model_file")
    else:
        port = filtered_argv[1]
        file_dir = filtered_argv[2]
        model_file = filtered_argv[3]

        mdp_executor = SimRobotPolicyExecutor(int(port), file_dir, model_file)
        mdp_executor.main()
