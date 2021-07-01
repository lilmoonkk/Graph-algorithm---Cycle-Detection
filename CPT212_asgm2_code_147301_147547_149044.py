# -*- coding: utf-8 -*-
"""
Created on Sun May 30 15:49:38 2021

@author: Moon
"""
import random
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx

distance_storage = np.array([[0,3235,1883,1012,19116],[3235,0,3198,2397,15881],
                               [1883,3198,0,1351,17671],[1012,2397,1351,0,18152],
                               [19116,15881,17671,18152,0]])

def GetKey(dictionary, value): 
    for key, item in dictionary.items():
        if item == value:
            return key

class Graph:
    def __init__(self):
        #initial graph
        self.graph = np.array([[0,3235,0,0,0],[0,0,3198,0,0],
                               [0,0,0,1351,0],[0,0,0,0,0],[19116,0,0,18152,0]])
        # MA: Marseille, France,YE: Yerevan, Armenia,OS: Orlo, Norway,
        # VI: Vienna Austria, WE: Wellington, New Zealand
        self.city = {0:"MA", 1:"YE", 2:"OS", 3:"VI", 4:"WE"}
        
    def DisplayGraph(self, title):
        rows, cols = np.where(self.graph != 0)
        weights=[] 
        for i in range(len(rows)):
            weights.append(self.graph[rows[i]][cols[i]])
        edges = zip(rows.tolist(),cols.tolist(),weights)
        g = nx.DiGraph()
        g.add_weighted_edges_from(edges)
        #Create positions of all nodes and save them
        pos = nx.circular_layout(g)
        #Draw the graph according to node positions
        nx.draw(g, pos, node_size=1000, labels=self.city, with_labels=True,
                connectionstyle='arc3, rad = 0.1',arrows=True)
        #Draw edge labels according to node positions
        nx.draw_networkx_edge_labels(g, pos, edge_labels=nx.get_edge_attributes(g,'weight'))
        plt.title(title)
        plt.legend(['MA: Marseille, France','YE: Yerevan, Armenia','OS: Orlo, Norway',
                    'VI: Vienna Austria', 'WE: Wellington, New Zealand'],
                    loc = 'lower right', fontsize = 6.5)
        plt.show()
        
    def RemoveEdge(self, sc, dc):
        #sc=source city    dc=destiniation city
        sc_index = GetKey(self.city, sc)
        dc_index = GetKey(self.city, dc)
        if self.graph[sc_index][dc_index] == 0 :
            print("The edge does not exist")
        else:
            self.graph[sc_index][dc_index] = 0
    
    def AddEdge(self):
        #sc=source city    dc=destiniation city
        #generate sc and dc
        valid = False
        while valid == False:
            sc_index = random.choice(list(self.city.keys()))
            dc_index = sc_index #to start while loop
            #make sure sc != dc
            while (dc_index == sc_index):
                dc_index = random.choice(list(self.city.keys()))
            if self.graph[sc_index][dc_index] == 0 :
                valid = True
        self.graph[sc_index][dc_index] = distance_storage[sc_index][dc_index]

# ==============Strongly Connected by Tan Hooi Ting===============================================
    #Function used by isSC() to perform the DFS
    def DFS(self, v, visited):
        #Mark the source vertex, v as visited
        visited[v] = True
            
        #Recur for all the unvisited vertices adjacent to the source vertex
        for i in range(5):
            if visited[i]==False:
                if self.graph[v][i]>0:
                    self.DFS(i, visited)
        
    #Function that return the transpose of the current graph               
    def getTranspose(self):
            
        #initialize a new graph
        transposeG = Graph()
            
        #Transpose the new graph
        for i in range(5):
            for j in range(5):
                transposeG.graph[i][j] = self.graph[j][i]
                    
        return transposeG
                    
    #Function that return true if the graph is strongly connected
    def isSC(self):
            
        #set all vertices as not visited
        visited = [False]*5
            
        #Start DFS traversal from the first vertex
        self.DFS(0, visited)
        
        for i in range(5):
            if visited[i] == False:
                return False
        
        #If the current graph is strongly connected, start traverse the transpose graph
        g = self.getTranspose()
        visited = [False]*5
            
        g.DFS(0,visited)
        
        for i in range(5):
            if visited[i] == False:
                return False
            
        return True
    
    #Function to generate strongly connected graph
    def StronglyConnected(self):
        
        SC = self.isSC()
        
        while SC == False:
            self.AddEdge()
            SC = self.isSC()
        
        if SC == True:
            self.DisplayGraph("Strongly Connected Graph")
            
# ==============DETECT CYCLE by Koo Xi Moon===================================

    def CheckCycle(self):
        visited = [False] * 5
        stack = [False] * 5
        edges = []
        for city in self.city:
            if visited[city] == False:
                if self.DFSFindCycle(city, visited, stack, edges):
                    self.DisplayCycle(edges)
                    return True
        return False
    
    # Depth First Search
    def DFSFindCycle(self, city, visited, stack, edges):
        visited[city] = True
        stack[city] = True
        #to find adjacent nodes
        neighbours = []
        items = self.graph[city,:]
        for i in range(5):
            if items[i] != 0:
                neighbours.append(i)
                
        #to reach adjacent node
        for neighbour in neighbours:
            if visited[neighbour] == False:
                if self.DFSFindCycle(neighbour, visited, stack, edges):
                    edges.append([city,neighbour])
                    return True
            #determinant
            #found the node if the adjacent node is already in stack 
            elif stack[neighbour] == True:
                edges.append([city,neighbour])
                return True
            
        stack[city] = False                
        return False
    
    def DisplayCycle(self, edges):
        #reverse edges
        edges = edges[::-1]
        #cut out edges that are not in the cycle
        intercept_node = edges[-1][-1]
        for i in range(len(edges)):
            if edges[i][0] == intercept_node:
                edges = edges[i:]
                break
        #flatten edges
        nodes =  sum(edges, [])
        city = {0:"MA", 1:"YE", 2:"OS", 3:"VI", 4:"WE"}
        #filter out city nodes that are not involved
        for place in self.city:
            if place not in nodes:
                city.pop(place)
        #start plotting graph
        g = nx.DiGraph()
        g.add_edges_from(edges)
        #Create positions of all nodes and save them
        pos = nx.circular_layout(g)
        #Draw the graph according to node positions
        nx.draw(g, pos, node_size=1000, labels=city, with_labels=True,
                connectionstyle='arc3, rad = 0.1',arrows=True)
        #Draw edge labels according to node positions
        plt.title("Detected Cycle")
        plt.show()
        
    # Depth First Search
    def DFSFindCycleWithCertainNode(self, city, visited, edges, node):
        visited[city] = True
        #to find adjacent nodes
        neighbours = []
        items = self.graph[city,:]
        for i in range(5):
            if items[i] != 0:
                neighbours.append(i)
                
        #to reach adjacent node
        for neighbour in neighbours:
            if visited[neighbour] == False:
                if self.DFSFindCycleWithCertainNode(neighbour, visited, edges, node):
                    edges.append([city,neighbour])
                    return True
            # determinant
            # found the cycle if the adjacent node is the chosen node
            elif neighbour == node:
                edges.append([city,neighbour])
                return True
            
        return False
    
    def GetCycleWithAppointedNode(self, node):
        visited = [False] * 5
        edges = []
        node = GetKey(self.city, node)
        while(self.DFSFindCycleWithCertainNode(node, visited, edges, node)==False):
            self.AddEdge()
            self.DisplayGraph("Current Graph")
            print("The current graph has no detected cycle with the node you are looking for.")
            print("No worries! We have added an edge for you.")
            input("Press any key to continue.\n")
            visited = [False] * 5
            edges = []
        self.DisplayCycle(edges)

# ==============SHORTEST PATH by Lee Hui Ying==================================
    # Function used for BFS to check if there is a path between 2 vertices 
    def check_reachable(self,sc,dc):
        
        
        #sc=source city    dc=destiniation city
        sc_index = GetKey(self.city, sc)
        dc_index = GetKey(self.city, dc)
        
        # Mark all the vertices is not visited yet
        visited = [False] * 5
        
        # Initialize a queue for BFS to store vertices which are not visited yet
        queue = []
        
        # Enqueue the source vertex into the queue and mark
        # it as visited
        queue.append(sc_index)
        visited[sc_index] = True
        
        # Keep checking if there is a vertex that has adjacent vertex which is 
        # not visited yet
        while queue:
            
            # Dequeue a vertex 'visit' from the queue 
            visit = queue.pop(0)
            
            # Check if the vertex 'visit' is the destination vertex, return true
            if visit == dc_index:
                return True
            
            # Else, do BFS
            # For all vertices adjacent to vertex 'visit' and the vertex is 
            # not visited yet, enqueue the vertex into the queue and mark it 
            # as vistied
            for i in range(5):
                if (self.graph[visit][i] != 0 and
                      (not visited[i])):
                    queue.append(i)
                    visited[i] = True
        # If finish doing BFS, but the destination vertex is not visited, return false
        return False
    
    # Function used to compute the shortest path from the source to destination
    # by using Dijkstra's algorithm
    def ShortestPath(self,sc,dc):
         
        row = len(self.graph)
        col = len(self.graph[0])
        
        
        # Initialize all distances as INFINITY
        D = [float("Inf")] * row
        
        # Initialize an array to store the shortest tree
        Shortest_tree = [-1]*row
        
        # Get the index of the source city and destinaiton city in the city dictionary
        sc_index = GetKey(self.city, sc)
        dc_index = GetKey(self.city, dc)
        
        # Assign the distance from source city to source city as 0
        D[sc_index] = 0
        
        # Create a priority queue and store all the vertices of the graph
        priority_q=[]
        for i in range(row):
            priority_q.append(i)
        
        # While the priority queue is not empty, find the shortest path for all vertices
        while priority_q:
            
            # Check the index of vertex with minimum distance from the priority queue
            u = self.minKey(D,priority_q)
            
            # Remove the vertex with minimum distance from the priority queue
            priority_q.remove(u)
            
            # Check if u is the index of destination then print the shortest path 
            # and shortest distance, then return
            if u == dc_index:
                print("\n Shortest distance = ", self.printPath(Shortest_tree, u))
                return 0
            
            # Cheeck all adjacent vertices of the dequeued vertex with index, u
            # If an adjacent vertex has not been visited and still in the priority queue, 
            # then get the weighted edge and compare the distance value in D[]
            # If the sum of distance of vertex with index,u is smaller than D[i],
            # Update the D[] value and the Shortest_tree[]
            for i in range(col):
                if(self.graph[u][i] != 0 and i in priority_q):
                    w = self.graph[u][i]
                    if((D[u] + w) < D[i]):
                        D[i] = D[u] + w
                        Shortest_tree[i]=u
    
    # Function used to return the index with minimum distance in the priority queue used
    # in the function ShortestPath
    def minKey(self,D,priority_q):
        # Initialize min value and min_index as -1
        minimum = float("Inf")
        min_index = -1
          
        # From the D array, use the for loop to check the minimum distance and 
        # return the index
        for i in range(len(D)):
            if D[i] < minimum and i in priority_q:
                minimum = D[i]
                min_index = i
        return min_index
    
    # Function used to print the shortest path in the Shortest_tree array using recursion
    # and return the value of shortest distance
    def printPath(self,Shortest_tree,u):
        row = len(self.graph)
        distance = 0
         
        #Base Case : If u is source
        # when Shortest_tree[-1] then path length = 0  
        if Shortest_tree[u] == -1 and u < row: 
            print ("\n Shortest path = ",self.city[u],end=" "),
            return distance
        
        # Recursion : print the vertex that the path go througH
        # and compute the shortest distance
        distance = distance_storage[u][Shortest_tree[u]]
        length = self.printPath(Shortest_tree, Shortest_tree[u])
        distance = length + distance
        print("->", end=" ")
        
  
        # print vertex only if its less than original vertex length.
        if u < row : 
            print (self.city[u],end=" "),
  
        return distance
#=============================================================================
    
class Menu:
    def __init__(self):
        self.G = Graph()
        self.G.DisplayGraph("Initial Graph")
        print("MA: Marseille, France")
        print("YE: Yerevan, Armenia")
        print("OS: Orlo, Norway")
        print("VI: Vienna Austria")
        print("WE: Wellington, New Zealand")
        
    def UserMenu(self):
        
        proceed = True
        while(proceed == True):
            
            print("\t===================Menu===================")
            print("\t   1. Cycle Detection")
            print("\t   2. Remove edges")
            print("\t   3. Display current graph")
            print("\t   4. Reset")
            print("\t   5. Exit\n")
            choice = input("\t   Please enter the number of your choice: ")
        
            if(choice == '1'):
                self.Function_1()
            elif(choice == '2'):
                self.Function_2()
            elif(choice == '3'):
                self.G.DisplayGraph("Current Graph")
            elif(choice == '4'):
                self.G.__init__()
                self.G.DisplayGraph("Current Graph")
            elif(choice == '5'):
                 proceed = False
            else:
                print("\n\t\t   Invalid input")
        
        return
            
        
    def Function_1(self):
        choice=input("Are you looking for\n1. Any Cycle\n2. Cycle with a node you are looking for\n")
        if(choice == '1'):
            while self.G.CheckCycle() == False :
                self.G.AddEdge()
                self.G.DisplayGraph("Current Graph")
                print("The current graph has no detected cycle.")
                print("No worries! We have added an edge for you.")
                input("Press any key to continue.\n")
        elif(choice == '2'):
            print("\nFrom")
            print("MA, YE, OS, VI, WE")
            node = input ("Enter the node :")
            if(node not in self.G.city.values()):
                print(node + " is not in the city list")
            else:
                self.G.GetCycleWithAppointedNode(node)
        else:
            print("\n\t\t   Invalid input")
           
                
    def Function_2(self):
        print("\nFrom")
        print("MA, YE, OS, VI, WE")
        sc = input ("Enter source city :")
        dc = input("Enter destination city : ")
        print("Source city = " + sc)
        print("Destination city = " + dc)
        # If the source city and the destination city are the same, return 0 length
        if (sc == dc):
            print("The source city and the destination city are the same city, the shortest distance is 0")
            return 0
        if(sc in self.G.city.values() and dc in self.G.city.values()):
        # Random generate edges between cities until there is a path between the two vertices        
            self.G.RemoveEdge(sc, dc)
            self.G.DisplayGraph("Current Graph")
        else:
            if(sc not in self.G.city.values()):
                print(sc + " is not in the city list")
            if(dc not in self.G.city.values()):
                print(dc + " is not in the city list")
    
                
#Run the program
UserMenu = Menu()
UserMenu.UserMenu()