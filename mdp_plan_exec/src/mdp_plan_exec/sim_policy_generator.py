from mdp_plan_exec.partial_sat_prism_java_talker import PartialSatPrismJavaTalker
import rospy
import os


class SimPolicyGenerator(object):
    def __init__(self, port, file_dir, file_name):

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

    def generate_time_bounded_spec(self, ltl_spec):
        return 'Pmax=?[ (' + ltl_spec + ') ]'

    # Returns the parsed policy obtained by prism, or None is there is an error generating the policy.
    def generate_policy_mdp(self, spec, use_time):
        if use_time:
            specification = self.generate_time_bounded_spec(spec)
        else:
            specification = self.generate_prism_specification(spec)
        prism_call_success = self.prism_policy_generator.call_prism(specification)
        if prism_call_success:
            return self.file_dir
        else:
            rospy.logwarn("Error generating policy. Aborting...")
            return None

    def shutdown_prism(self, remove_dir):
        self.prism_policy_generator.shutdown(remove_dir)
