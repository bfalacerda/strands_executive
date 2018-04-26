from mdp_plan_exec.partial_sat_prism_java_talker import PartialSatPrismJavaTalker
import rospy
import os


class SimPolicyGenerator(object):
    def __init__(self, port, file_dir, file_name, mdp):

        self.mdp = mdp
        self.current_nav_policy_state_defs = {}
        self.file_dir = file_dir
        self.file_name = file_name
        try:
            os.makedirs(file_dir)
        except OSError as ex:
            print 'error creating PRISM directory:', ex
        self.prism_policy_generator = PartialSatPrismJavaTalker(port, file_dir, file_name)

    def generate_prism_specification(self, ltl_spec):
        return 'partial(R{"time"}min=? [ (' + ltl_spec + ') ])'

    # Returns the parsed policy obtained by prism, or None is there is an error generating the policy.
    def generate_policy_mdp(self, spec, initial_waypoint, epoch):
        specification = self.generate_prism_specification(spec.ltl_task)
        rospy.loginfo("The specification is: " + specification)
        self.mdp.create_top_map_mdp_structure()
        self.mdp.add_extra_domain(spec.vars, spec.actions)

        # update initial state
        self.mdp.set_initial_state_from_waypoint(initial_waypoint)
        self.mdp.add_predictions(self.file_dir + self.file_name, epoch)

        prism_call_success = self.prism_policy_generator.call_prism(specification)
        if prism_call_success:
            return self.file_dir
        else:
            rospy.logwarn("Error generating policy. Aborting...")
            return None

    def shutdown_prism(self, remove_dir):
        self.prism_policy_generator.shutdown(remove_dir)
