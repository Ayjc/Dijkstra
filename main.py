import heapq
import math

from search.algorithms import State
from search.map import Map
import getopt
import sys

def main():
    """
    Function for testing your A* and Dijkstra's implementation. There is no need to edit this file.
    Run it with a -help option to see the options available. 
    """
    optlist, _ = getopt.getopt(sys.argv[1:], 'h:m:r:', ['testinstances', 'plots', 'help'])

    plots = True
    for o, a in optlist:
        if o in ("-help"):
            print("Examples of Usage:")
            print("Solve set of test instances: main.py --testinstances")
            print("Solve set of test instances and generate plots: main.py --testinstances --plots")
            exit()
        elif o in ("--plots"):
            plots = True
        elif o in ("--testinstances"):
            test_instances = "test-instances/testinstances.txt"
                              
    gridded_map = Map("dao-map/brc000d.map")
    
    nodes_expanded_dijkstra = []    
    nodes_expanded_bibs = []
    
    start_states = []
    goal_states = []
    solution_costs = []
       
    file = open("test-instances/testinstances.txt", "r")
    for instance_string in file:
        list_instance = instance_string.split(",")
        start_states.append(State(int(list_instance[0]), int(list_instance[1])))
        goal_states.append(State(int(list_instance[2]), int(list_instance[3])))
        
        solution_costs.append(float(list_instance[4]))
    file.close()
        
    for i in range(0, len(start_states)):    
        start = start_states[i]
        goal = goal_states[i]
    
        cost, expanded_dijskstra, closeD = Dijkstra_algorithm(start, goal, gridded_map) # Implement here the call to your Dijkstra's implementation for start, goal, and gridded_map

        nodes_expanded_dijkstra.append(expanded_dijskstra)

        gridded_map.plot_map(closeD, start, goal, "Dijkstra " + str(i + 1))

        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by Dijkstra and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()

        cost, expanded_astar, closeB = BiBs_algorithm(start, goal, gridded_map) # Implement here the call to your Bi-BS's implementation for start, goal, and gridded_map

        nodes_expanded_bibs.append(expanded_astar)

        gridded_map.plot_map(closeB, start, goal, "BiBS " + str(i + 1))
        
        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by Bi-HS and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()
    
    if plots:
        from search.plot_results import PlotResults
        plotter = PlotResults()
        plotter.plot_results(nodes_expanded_bibs, nodes_expanded_dijkstra, "Nodes Expanded (Bi-BS)", "Nodes Expanded (Dijkstra)", "nodes_expanded")
    
    print('Finished running all experiments.')

def Dijkstra_algorithm (init_s, goal_s, map):
    open_list = [] #Initializing the open list
    closed = {init_s.state_hash(): init_s} #Initializing the closed list
    count_num_nodes = 0 #Initializing node counting variable
    heapq.heappush(open_list, init_s) #Pushing the init state into the heap

    while open_list:
        temp_node = heapq.heappop(open_list) #Getting the initi state from the heap

        count_num_nodes += 1 #Every time poping the node add one to the counting node variable

        if temp_node == goal_s: #Means we found the goal state, return the cost of the goal state, num of nodes and closed list
            return temp_node.get_g(), count_num_nodes, closed

        for node_prime in map.successors(temp_node):
            node_prime_hash_value = node_prime.state_hash() #Calculating the node prime hash table value

            if node_prime_hash_value not in closed: #checking whether we have this in record or not
                heapq.heappush(open_list, node_prime)
                closed[node_prime_hash_value] = node_prime

            elif node_prime_hash_value in closed.keys() and node_prime.get_g() < closed[node_prime_hash_value].get_g():
                #Comparing the cost of the node prime with the one that is already in the closed list.
                #if the cost of node prime in current iteration is more optimal then we update the cost.
                if node_prime in open_list:
                    open_list[open_list.index(node_prime)] = node_prime
                    heapq.heapify(open_list)
                closed[node_prime_hash_value] = node_prime

    return -1, count_num_nodes, closed #if we didn't find a solution, return -1 as the cost


def BiBs_algorithm(init_s, goal_s, map):
    #Initializing the open list for both forward and backward direction
    open_list_forward = []
    open_list_backward = []
    ##Initializing the closed list for both forward and backward direction
    closed_forward = {init_s.state_hash(): init_s}
    closed_backward = {goal_s.state_hash(): goal_s}
    ##Pushing the init state into the heap in both direction
    heapq.heappush(open_list_forward, init_s)
    heapq.heappush(open_list_backward, goal_s)
    count_num_nodes = 0
    u_cost = math.inf #Initializing the cost with positive infinity

    while open_list_forward and open_list_backward:

        count_num_nodes += 1  # Every time in the iteration the node add one to the counting node variable

        if u_cost <= (open_list_forward[0].get_g() + open_list_backward[0].get_g()):
            closed_forward.update(closed_backward) #Means we found an optimal solution, and we combine the two closed list (dictionary) into one
            return u_cost, count_num_nodes, closed_forward

        if open_list_forward[0].get_g() < open_list_backward[0].get_g(): #Deciding which direction should we start first

            current_node = heapq.heappop(open_list_forward)

            for node_pri in map.successors(current_node):

                node_prime_hash = node_pri.state_hash()  # Calculating the node prime hash table value

                if node_prime_hash in closed_backward:  # checking whether we have this in backward dictionary or not
                    u_cost = min(u_cost, (node_pri.get_g() + closed_backward[node_prime_hash].get_g()))

                if node_prime_hash not in closed_forward:
                    heapq.heappush(open_list_forward, node_pri)
                    closed_forward[node_prime_hash] = node_pri

                if node_prime_hash in closed_forward.keys() and node_pri.get_g() < closed_forward[node_prime_hash].get_g():
                    # Comparing the cost of the node prime with the one that is already in the closed list.
                    # if the cost of node prime in current iteration is more optimal we update the cost.
                    if node_pri in open_list_forward:
                        open_list_forward[open_list_forward.index(node_pri)] = node_pri
                        heapq.heapify(open_list_forward)
                    closed_forward[node_prime_hash] = node_pri
        else:
            #Start searching from the backward direction
            current_node = heapq.heappop(open_list_backward)

            for node_pri in map.successors(current_node):
                node_prime_hash = node_pri.state_hash()  # Calculating the node prime hash table value

                if node_prime_hash in closed_forward:  # checking whether we have this in forward dictionary or not
                    # Applying the inequality to check whether the u cost is optimal or not
                    u_cost = min(u_cost, (node_pri.get_g() + closed_forward[node_prime_hash].get_g()))


                if node_prime_hash not in closed_backward:
                    heapq.heappush(open_list_backward, node_pri)
                    closed_backward[node_prime_hash] = node_pri

                if node_prime_hash in closed_backward.keys() and node_pri.get_g() < closed_backward[node_prime_hash].get_g():
                    # Comparing the cost of the node prime with the one that is already in the closed list.
                    # if the cost of node prime in current iteration is more optimal we update the cost.
                    if node_pri in open_list_backward:
                        open_list_backward[open_list_backward.index(node_pri)] = node_pri
                        heapq.heapify(open_list_backward)
                    closed_backward[node_prime_hash] = node_pri

    closed_forward.update(closed_backward) #if we didn't find a solution, still need to combine the two forward and backward dictionary into one
    return -1, count_num_nodes, closed_forward  #if we didn't find a solution, return -1 as the cost



if __name__ == "__main__":
    main()