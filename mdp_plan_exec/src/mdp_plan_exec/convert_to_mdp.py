from rapport.stats import discretise_by_uniform_x

def convert_to_mdp_time_as_reward(tm, save_loc, initial_wp):
    nodes = tm.nodes.keys()
    init_index = nodes.index(initial_wp)
    with open(save_loc, 'w') as mdp_file:
        mdp_file.write('mdp\n\n')
        mdp_file.write('module M\n\n')
        mdp_file.write('waypoint:[-1..{}] init {};\n\n'.format(len(nodes)-1, init_index))
        for action in tm.edges.keys():
            edge = tm.edges[action]
            model = edge.traversal_model.get_discrete_duration_model()
            oc = []
            # print(model._components)
            for tup in model._components:
                if tup[0] == 'success':
                    print(tup)
                    oc.append('{}:(waypoint\'={})'.format(tup[1], nodes.index(edge.end.name)))
                else:
                    oc.append('{}:(waypoint\'={})'.format(tup[1], -1))
            mdp_file.write('[{}] (waypoint={}) -> {};\n'.format(action, nodes.index(edge.start.name), "+".join(oc)))
        mdp_file.write('\nendmodule\n\n')

        for i in range(len(nodes)):
            mdp_file.write('label "{}" = (waypoint={});\n'.format(nodes[i], i))

        mdp_file.write('\nrewards "time"\n')
        for action in tm.edges.keys():
            edge = tm.edges[action]
            mean = edge.traversal_model.get_continuous_duration_model().expect()
            mdp_file.write('    [{}] (waypoint={}):{};\n'.format(action, nodes.index(edge.start.name), mean))

        mdp_file.write('endrewards')

def convert_to_time_mdp(tm, save_loc, initial_wp):
    nodes = tm.nodes.keys()
    init_index = nodes.index(initial_wp)
    with open(save_loc, 'w') as mdp_file:
        mdp_file.write('mdp\n\n')
        mdp_file.write('module M\n\n')
        mdp_file.write('waypoint:[-1..{}] init {};\n\n'.format(len(nodes)-1, init_index))
        T=500
        mdp_file.write('time:[0..{}] init 0;\n\n'.format(T))
        for action in tm.edges.keys():
            edge = tm.edges[action]
            def disc(rv):
                return discretise_by_uniform_x(rv, n_values=4)
            model = edge.traversal_model.get_discrete_duration_model(disc)
            oc = []
            # print(model._components)
            for tup in model._components:
                if tup[0] == 'success':
                    for i in range(len(tup[2].expected_values)):
                        oc.append('{}:(waypoint\'={}) & (time\'=min({}+1,time+{}))'.format(tup[1]*tup[2].pmf_probabilities[i], nodes.index(edge.end.name), T, int(round(tup[2].expected_values[i]))))
                else:
                    for i in range(len(tup[2].expected_values)):
                        oc.append('{}:(waypoint\'={}) & (time\'=min({}+1,time+{}))'.format(tup[1]*tup[2].pmf_probabilities[i], -1, T, int(round(tup[2].expected_values[i]))))

            mdp_file.write('[{}] (waypoint={}) -> {};\n'.format(action, nodes.index(edge.start.name), "+".join(oc)))
        mdp_file.write('\nendmodule\n\n')

        for i in range(len(nodes)):
            mdp_file.write('label "{}" = (waypoint={});\n'.format(nodes[i], i))

        mdp_file.write('\nrewards "time"\n')
        for action in tm.edges.keys():
            edge = tm.edges[action]
            mean = edge.traversal_model.get_continuous_duration_model().expect()
            mdp_file.write('    [{}] (waypoint={}):{};\n'.format(action, nodes.index(edge.start.name), mean))

        mdp_file.write('endrewards')
