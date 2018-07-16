from mdp_plan_exec.ros_free_sim import SimulatorNoRos
import matplotlib.pyplot as plt

if __name__ == '__main__':
    simulator = SimulatorNoRos(8088, "saved_prism/", "test_mdp.mdp", False, True)
    time_limit = 100
    results = simulator.execute_policy("WayPoint6", "WayPoint2", time_limit)
    goal_index = results[4]
    fails=[]
    time_fails=[]
    successes=[]
    for i in range(len(results[1])):
        result = results[1][i]
        if results[3][i][1] != goal_index:
            fails.append((i,result))
        elif result > time_limit:
            time_fails.append((i,result))
        else:
            successes.append((i, result))
    plt.xlabel("Iteration")
    plt.ylabel("Duration")
    plt.scatter(*zip(*successes), color='g')
    plt.scatter(*zip(*fails), color='r')
    plt.scatter(*zip(*time_fails), color='b')
    plt.legend(('Succeded', 'Failed to reach objective', 'Failed by exceeding time limit'))
    plt.show()
