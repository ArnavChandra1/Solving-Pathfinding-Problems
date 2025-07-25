import time
from search.algorithms import Dijkstra, AStar, State
from search.map import Map
import getopt
import sys

def verify_path(start, goal, path, map):
    if path is None:
        return True

    if not (start == path[0]) or not (goal == path[-1]):
        return False

    for i in range(len(path) - 1):
        current = path[i]
        children = map.successors(current)
        
        contains_next = False
        for child in children:
            if child == path[i + 1]:
                contains_next = True
                break

        if not contains_next:
            return False
    return True
        

def main():
    """
    Function for testing your A* and Dijkstra's implementation. There is no need to edit this file.
    Run it with a -help option to see the options available. 
    """
    test_instances = "test-instances/testinstances.txt"
                              
    gridded_map = Map("dao-map/brc000d.map")
    dijkstra = Dijkstra(gridded_map)
    astar = AStar(gridded_map)
    
    nodes_expanded_dijkstra = []  
    nodes_expanded_astar = []

    time_dijkstra = []  
    time_astar = []

    start_states = []
    goal_states = []
    solution_costs = []

    file = open(test_instances, "r")
    for instance_string in file:
        list_instance = instance_string.split(",")
        start_states.append(State(int(list_instance[0]), int(list_instance[1])))
        goal_states.append(State(int(list_instance[2]), int(list_instance[3])))
        
        solution_costs.append(float(list_instance[4]))
    file.close()
        
    for i in range(0, len(start_states)):    
        print("Working on case ", i)
        start = start_states[i]
        goal = goal_states[i]
    
        time_start = time.time()
        path, cost, expanded_diskstra = dijkstra.run(start, goal)
        time_end = time.time()
        nodes_expanded_dijkstra.append(expanded_diskstra)
        time_dijkstra.append(time_end - time_start)
        verified_path = verify_path(start, goal, path, gridded_map)

        if cost != solution_costs[i] or not verified_path:
            print("There is a mismatch in the solution cost found by Dijkstra and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print("Is the path correct?", verified_path)
            print()    

        start = start_states[i]
        goal = goal_states[i]
    
        time_start = time.time()
        path, cost, expanded_astar = astar.run(start, goal)
        time_end = time.time()

        nodes_expanded_astar.append(expanded_astar)
        time_astar.append(time_end - time_start)

        verified_path = verify_path(start, goal, path, gridded_map)
        if cost != solution_costs[i] or not verified_path:
            print("There is a mismatch in the solution cost found by A* and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print("Is the path correct?", verified_path)
            print()

        gridded_map.plot_map(dijkstra.get_closed_data(), start, goal, 'solution-maps/dijkstra_' + str(i + 1))
        gridded_map.plot_map(astar.get_closed_data(), start, goal, 'solution-maps/astar_' + str(i + 1))


    from search.plot_results import PlotResults
    plotter = PlotResults()
    plotter.plot_results(nodes_expanded_astar, nodes_expanded_dijkstra, "Nodes Expanded (A*)", "Nodes Expanded (Dijkstra)", "nodes_expanded")
    plotter.plot_results(time_astar, time_dijkstra, "Running Time (A*)", "Running Time (Dijkstra)", "running_time")

if __name__ == "__main__":
    main()