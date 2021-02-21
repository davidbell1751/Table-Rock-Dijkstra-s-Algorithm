# DIJKSTRAS ALGORTIHM FOR FIND THE COST OF TRANERSING THE TRAILS ON TABLE ROCK VIA THE SHORTEST PATH 
# Python Algorthim Bootcamp from Galvanize Coding  
# This algorithm is best for static path that isn't going to change like a router path would 
# David Bell
# February 2021
# CS 499

# The shortest path = the hardest path

    
           # Initialization of the algorithm - Description of what the algrotihm will do 
           # dijkstras algortithm for Table Rock Mountain Hiking Paths
           # The Cost from a node to the same node will be zero
           # Create a table showing the costs from each node
           #Let distance to all other nodes = 10**9 (e.g. infinity)
           # Repeat:
            #    Visit the unvisited node with the smallest known distance from this start node
            #        For current node, examine its unvisited neighbors
             #       For current node, calc the distance of each neighbor from this start node
             #       if the calc'd distance of a node is less than the current distance in the cost_table, update the shortest distance in the cost table
            #        Update the previous node for each updated distance
              #      Add the current node to the list of visited nodes and remove it from the list of unvisited nodes
          #  Until all nodes are visited
          


class Graph:                                        # class for creating the graph method
    def __init__(self, nodes_in_graph=[]):             # initialize method passing self and nodes in graph defaulted to empty list
        self.nodes_in_graph = nodes_in_graph            # default the nodes in graph variable to empty list

    def add_node(self, node):                         # method add node   / 
        self.nodes_in_graph.append(node)                # append the node to nodes in graph

    def add_nodes(self, node_list):                     # add multiple nodes with list of the nodes / if there are multiple nodes in a list then it will pass in each additional node
        for node in node_list:                          # pass node objects
            self.add_node(node)                         # use the method self.add_node to add node

    def get_node(self, nodename):                       # using a loop to find if the node and nodename are equivilant and if so return the node
        for node in self.nodes_in_graph:
            if node.nodename == nodename:
                return node

    def build_cost_tables(self):                        # method build cost table
        for node in self.nodes_in_graph:                
            node.build_cost_table(self)

    def get_shortest_path(self, from_nodename, to_nodename):
        from_node = self.get_node(from_nodename)
        return from_node.get_shortest_path_to(to_nodename)

class Node:
    def __init__(self, nodename, connections):      # initialize node  and pass initialization arguments    add self reference for any method in the class  initialize the dictionary of the connections
        self.nodename = nodename                    # variable to get nodename
        self.connections = connections              # variable to get connections dictionary

        self.cost_table = {self.nodename: {'cost': 0, 'previous': None}}    # cost table as dictionary nodename is the key for the dictonary items cost and previous  the cost to go from a node to itself is none or zero

        for name, cost in self.connections.items():                          # traverse the connections dictionary
            self.cost_table[name] = {'cost': cost, 'previous': self.nodename}    # add values into the cost table  for each nodename add cost to each connection

    def __repr__(self):                                                 # output cost table
        cost_table_header = '\tNode\tCost\tPrevious\n'                  # header for the cost table interface with line break
        cost_table_str = ''.join([f'\t{node}\t{vals["cost"]}\t{vals["previous"]}\n' for node, vals in self.cost_table.items()])     # f string with list of strings and joined together to create table  use double quotes / single will fail
                                                                                                                                    # nodes and vals in instead of keys and values // for node the value is a dictionary
        return f'nodename: {self.nodename}\nconnections: {self.connections}\ncost_table:\n {cost_table_header}{cost_table_str}'     # return which node this is

    def __str__(self):
        cost_table_header = '\tNode\tCost\tPrevious\n'
        cost_table_str = ''.join([f'\t{node}\t{vals["cost"]}\t{vals["previous"]}\n' for node, vals in self.cost_table.items()])

        return f'nodename: {self.nodename}\nconnections: {self.connections}\ncost_table:\n {cost_table_header}{cost_table_str}'

    def get_shortest_path_to(self, some_node_name):                 # shortest path function
        shortest_path = []                                          # nodes go in path in reverse order
        costs = []                                                  # the costs affiliated with those nodes
        total_cost = self.cost_table[some_node_name]['cost']        # total cost for the cost table
        current_node = some_node_name[:]                            # show the current working node in the table

        for _ in range(10000):                                      # not more then 10000 nodes
            shortest_path.append(current_node)                      # append to the shortest path the current node

            if self.cost_table[current_node]['previous'] != None:           # if the previous node isn't equal to none then append the cost
                costs.append(self.cost_table[current_node]['cost'] - self.cost_table[self.cost_table[current_node]['previous']]['cost'])   # take the cost from the current node and subtract the cost of the previous node
    
            current_node = self.cost_table[current_node]['previous']            # update the current node 

            if current_node == None:                                # break if the current node is none   exit condition
                break

        costs = list(reversed([str(cost) for cost in costs])) # stringify    convert costs values to strings   reverse the object and cast to a list

        return_str = ''                 # return string

        for idx, let in enumerate(reversed(shortest_path)):                 # embeds the cost in the path   enumberate the reversed version of the shortest path
            if idx < len(costs):                                            # if the index is less then the length of the costs 
                return_str += let + f' -{costs[idx]}-> '                    # take the node letter and interpolate the associated cost
            else:                               
                return_str += let                                           # if the index is not less then the length of the costs
            
        return_str += f'\ntotal cost:{total_cost}'                          # append the total cost

        return return_str                                                   # return the return string

    def build_cost_table(self, graph):          # method for building cost table


    
        visited = [self.nodename]               # visited the node once its initialized

      
        for node in graph.nodes_in_graph:                               # Populate cost table entries with remaining nodes in the graph
            if node.nodename not in self.cost_table:                       # Let distance to all other nodes = 10**9 which represents infinity
                self.cost_table[node.nodename] = {'cost': 10**9, 'previous': None}      # we have not added the node to the cost table  exclude self name and exclude the adjacent nodes

        # Populate unvisited list with all nodenames
        unvisited = [node.nodename for node in graph.nodes_in_graph if node.nodename != self.nodename]      # != self.nodename to exclude adding unvisited self node

        # instead of using a while loop, let's assume a max of N node in unvisited and create an exit condition
        # step through the nodes, prioritizing the lowest cost nodes, and updating the cost table 
        for _ in range(100):                                #  not more then 100 iterations of this loop
            # search for node with lowest cost
            min_cost, min_node = min([(self.cost_table[nodename]['cost'], nodename) for nodename in unvisited])   # using min function by looking at the cost table using a sorted tuple 
                                                                                                                # first value in the tuple is the cost / the cost will be placed in the min_cost variable and the node in the min_node variable

            min_node = graph.get_node(min_node)         # get the node from a method on graph

            for nodename, cost in min_node.connections.items():           # nodename is the key and cost is the value with connections as the dictionary  
                dist_from_start = min_cost + cost                           # the distance from the starting node will equal the minimum cost plus the cost

                if dist_from_start < self.cost_table[nodename]['cost']:         # if the distance from the start is less then the cost table
                    self.cost_table[nodename]['cost'] = dist_from_start         # if the calculated distance is less then what we see in the table then update the table
                    self.cost_table[nodename]['previous'] = min_node.nodename       # update the previous node as well to get the node name of the minimum node

                visited.append(min_node.nodename)               # if we visited the node then append it

                if min_node.nodename in unvisited:              # if that node name is in unvisited then we want to remove it from unvisited 
                    unvisited.remove(min_node.nodename)         # removed and won't be checked again during for loop
            
            if len(unvisited) == 0:                             # exit condition   once length of zero is reached then we are done and can break out of the for loop
                break
        
if __name__ == '__main__':
    # NODE DEFINITIONS
    A = Node('A', {'B': 5, 'C': 7, 'D': 2})   # dictionary of all connections
    B = Node('B', {'A': 5, 'E': 4})
    C = Node('C', {'A': 7, 'D': 3, 'F': 5})
    D = Node('D', {'A': 2, 'C': 3, 'E': 4, 'G': 6})
    E = Node('E', {'B': 4, 'D': 4, 'G': 2})
    F = Node('F', {'C': 5, 'G': 7, 'H': 6 })
    G = Node('G', {'D': 6, 'E': 2, 'F': 7, 'H': 3})
    H = Node('H', {'F': 6, 'G': 3})


    

    graph = Graph()                                     # graph constructor
    graph.add_nodes([A, B, C, D, E, F, G, H])           # method to add nodes

    A.build_cost_table(graph)
    print(A)

    graph.build_cost_tables()               # build cost table method
    print(graph.nodes_in_graph)
    
    print(A.get_shortest_path_to('H'))
    print(B.get_shortest_path_to('H'))

    print(graph.get_shortest_path('A', 'H'))
    print(graph.get_shortest_path('B', 'H'))