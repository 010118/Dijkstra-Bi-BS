from search.algorithms import State
from search.map import Map
import getopt
import sys
import heapq
import math

def Dijkstra(start,goal,gridded_map):
    OPEN = []
    CLOSED = {}
    CLOSED[start.state_hash()] = start
    nodes = 0
    heapq.heappush(OPEN,start)

    while len(OPEN)!=0: # 如果长度不等于0
        pre_node = heapq.heappop(OPEN)
        nodes += 1
        if pre_node == goal:
            return pre_node.get_g(), nodes,CLOSED
            # 上面 是 stop 了
        for child in gridded_map.successors(pre_node):
            hash_value = child.state_hash()

            if hash_value not in CLOSED:
                CLOSED[hash_value] = child
                heapq.heappush(OPEN, child)

            if hash_value in CLOSED and child.get_g() < CLOSED[hash_value].get_g():
                CLOSED[hash_value] = child
                heapq.heappush(OPEN, child)
                if child in OPEN:
                    OPEN[OPEN.index(child)] = child
                    heapq.heapify(OPEN)

    return -1, nodes, CLOSED


def BiBs(start, goal, gridded_map):
    OPENf = []
    OPENB = []
    closedf = {start.state_hash():start}
    closedb = {goal.state_hash():goal}

    exp_nodes = 0
    heapq.heappush(OPENf, start)
    heapq.heappush(OPENB, goal)
    U = math.inf

    while len(OPENf) != 0 and len(OPENB) != 0:
         exp_nodes += 1
         if U <= (OPENf[0].get_g()+OPENB[0].get_g()):
           return U, exp_nodes, closedf

         if OPENf[0].get_g() < OPENB[0].get_g():
             n = heapq.heappop(OPENf)
             for child in gridded_map.successors(n):
                 child_hash = child.state_hash()
                 if child_hash in closedb:
                     U = min(U, child.get_g() + closedb[child_hash].get_g())
                 if child_hash not in closedf:
                     heapq.heappush(OPENf, child)
                     closedf[child_hash] = child
                 if child_hash in closedf and child.get_g() < closedf[child_hash].get_g():

                     closedf[child_hash] = child
                     if child in OPENf:
                         OPENf[OPENf.index(child)] = child
                         heapq.heapify(OPENf)
         else:
             n = heapq.heappop(OPENB)
             for child in gridded_map.successors(n):
                 child_hash = child.state_hash()
                 if child_hash in closedf:
                     U = min(U, closedf[child_hash].get_g() + child.get_g())
                 if child_hash not in closedb:
                     heapq.heappush(OPENB, child)
                     closedb[child_hash] = child
                 if child_hash in closedb and child.get_g() < closedb[child_hash].get_g():
                     closedb[child_hash] = child
                     if child in OPENB:
                         OPENB[OPENB.index(child)] = child
                         heapq.heapify(OPENB)

    closedf.update(closedb)

    return -1, exp_nodes,closedf








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
        #elif o in ("--testinstances"):
    test_instances = "test-instances/testinstances.txt"
                              
    gridded_map = Map("dao-map/brc000d.map")
    
    nodes_expanded_dijkstra = []    
    nodes_expanded_bibs = []
    
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
        start = start_states[i]
        goal = goal_states[i]
    
        cost, expanded_diskstra, closeD = Dijkstra(start,goal,gridded_map) # Implement here the call to your Dijkstra's implementation for start, goal, and gridded_map
        nodes_expanded_dijkstra.append(expanded_diskstra)
        gridded_map.plot_map(closeD, start, goal, "Dijkstra" + str(i + 1))
        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by Dijkstra and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()

        cost, expanded_astar, closeB =BiBs(start,goal,gridded_map) # Implement here the call to your Bi-BS's implementation for start, goal, and gridded_map
        gridded_map.plot_map(closeB, start, goal, "BiBs"+str(i+1))
        nodes_expanded_bibs.append(expanded_astar)
        
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
        plotter.plot_results(nodes_expanded_bibs, nodes_expanded_dijkstra, "Nodes Expanded (Bi-HS)", "Nodes Expanded (Dijkstra)", "nodes_expanded")
    
    print('Finished running all experiments.')

if __name__ == "__main__":
    main()