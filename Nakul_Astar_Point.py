# -*- coding: utf-8 -*-
"""
Created on Tue Apr  2 03:27:10 2019

@author: nakul
"""

# -*- coding: utf-8 -*-
"""
Created on Sat Mar 30 13:03:01 2019

@author: nakul
"""

import numpy as np
import matplotlib.pyplot as plt
import heapq
import math
import time

## CODE FOR OBSTACLE MAPPING =================================================
# For boundary
def check_boundary(x, y):
    bool1= (x==0 ) or ( x == 249) 
    bool2= (y==0 ) or ( y == 149) 
    req = False    
    if (bool1 or bool2):
        req = True
    return req
# For rectangle
def check_rect(x, y):
    x >= 50
    x <= 100
    y >= 67.5
    y <= 112.5
    req = False
    # using half-planes to define obstacle space
    if (x >= 50 and x <= 100 and y >= 67.5 and y <= 112.5):
        req = True
    return req
# For circle
def check_circle(x, y):
    eqn_circle = (x - 190)**2 + (y - 130)**2 - 225
    req = False
    # using semi-algabraic equation to define obstacle space
    if (eqn_circle <= 0):
        req = True
    return req
# For ellipse
def check_ellipse(x, y):
    eqn_ellipse = ((x - 140)**2)/225 + ((y - 120)**2)/36 - 1
    req = False
    # using semi-algabraic equation to define obstacle space
    if (eqn_ellipse <= 0):
        req = True
    return req
# For polygon 
def check_polygon(x, y):
    eqn1 = y - 52
    eqn2 = 15 - y
    eqn3 = 37*x -20*y - 6101
    eqn4 = -(37*x) + (13*y) + 5355
    
    eqn5 = 52 - y
    eqn6 = 38*x + 23*y - 8530 
    eqn7 = -38*x + 7*y + 5830
    
    eqn8 = 2*x + 19*y - 1314
    eqn9 = -41*x -25*y + 6525
    eqn10 = 37*x - 13*y - 5355
    
    bool1 = (eqn1 <= 0 and eqn2 <= 0 and eqn3 <=0 and eqn4 <= 0)
    bool2 = (eqn5 <= 0 and eqn6 <= 0 and eqn7 <=0)
    bool3 = (eqn8 <= 0 and eqn9 <= 0 and eqn10 <=0)
    req = False
    if ( bool1 or bool2 or bool3):
        req = True
    return req
#==============================================================================
            
flag_for_display = True

sx = int(input("Enter x coordinate of start point:"))
sy = int(input("Enter y coordinate of start point:"))
gx = int(input("Enter x coordinate of goal point:"))
gy = int(input("Enter y coordinate of goal point:"))
print("\nThe radius and clearance for point robot has been considered zero.")
startp = (sx, sy)
goalp = (gx, gy)
print("Start:", startp)
print("Goal:", goalp)

ox=[]
oy=[]
for i in range(0,250):
    for j in range(0,150):
        req0 = check_boundary(i, j)
        req1 = check_rect(i, j)
        req2 = check_circle(i, j)
        req3 = check_ellipse(i, j)
        req4 = check_polygon(i, j)        
        if (req0 or req1 or req2 or req3 or req4):
            ox.append(i)
            oy.append(j)
            
plt.plot(ox,oy,"ko")
plt.plot(sx,sy,"go")
plt.plot(gx,gy,"ro")
#plt.grid(True)
plt.axis("equal")
plt.show()
plt.ylim((-5, 155))
plt.xlim((-5, 255))  


start = time.time()

class Node:
    def __init__(self, x, y, gcost,fcost, parent):
        self.x = x
        self.y = y
        self.gcost = gcost
        self.fcost = fcost
        self.parent = parent 
    def tuple(self):
        return (self.fcost,self.gcost,self.x,self.y,self.parent)

def visited_index(node_tuple):
    k = 250*node_tuple[2] + node_tuple[3]
    return k
    
def heuristic(rx, ry, gx, gy):
    euc_dist = math.sqrt((rx - gx)**2 + (ry - gy)**2)
    return euc_dist
    
def astar(sx, sy, gx, gy, ox, oy):
    # Stores all nodes initially
    unvisited = []
    heapq.heapify(unvisited)
    # Stores visited nodes in dictionary
    visited = dict() 
    
    temp_unvisited_keys = dict()                           #####ADDED
    
    s_h = round(heuristic(sx, sy, gx, gy)*10)
    start_node = Node(sx,sy,0,0,0)
    goal_node = Node(gx,gy,0,0,0)
    
    heapq.heappush(unvisited, start_node.tuple())
    
    x = visited_index(start_node.tuple())                  #####ADDED
    temp_unvisited_keys[x] = start_node.tuple()            #####ADDED
    
    expand_nodes = [[0,1,10],[1,1,14],[1,0,10],[1,-1,14],
                    [0,-1,10],[-1,-1,14],[-1,0,10],[-1,1,14]]
    
    # Loop for node exporation
    j = 0
    nodList=[]
    #len(unvisited)!=0
    while(len(unvisited)!=0):                 #=======================================
        # current node will be one with minimum cost
        #print("\n\nloop",j)
        current_node = heapq.heappop(unvisited)
        nodList.append(current_node)
        
        key_current = visited_index(current_node)           #####ADDED
        #print(key_current)
        del temp_unvisited_keys[key_current]                #####ADDED
        #print("Current",current_node)
        #print("Unvisited after pop",unvisited)
        #print("visited", visited)
        #print("Keys:  ", temp_unvisited_keys ,"\n")
         
        if flag_for_display:               #=======================================
            plt.plot(current_node[2] , current_node[3], "xr")
            #len(newList)%1000==0
            if (len(nodList)%1000 ==0 or len(nodList) == 1):
                plt.pause(0.01)  
                    
        # Stop if the goal is found 
        if current_node[2] == goal_node.x and current_node[3] == goal_node.y:
            print("\n\nCongrats! You have reached to goal")
            goal_node.parent = current_node[4]
            goal_node.cost = current_node[0]
            break
        
        
        for i in range(0, len(expand_nodes)):  #=======================================
            new_x = current_node[2] + expand_nodes[i][0]
            new_y = current_node[3] + expand_nodes[i][1]
            h = round(heuristic(new_x,new_y, gx, gy)*10)
            new_gcost = current_node[1] + expand_nodes[i][2]  
            new_fcost = new_gcost+ (h)
            new_parent = key_current
            
            node_obj = Node(new_x, new_y, new_gcost, new_fcost, new_parent)
            node_key = visited_index(node_obj.tuple())
            #print(node_key, node_obj.tuple())
            
            # condition for checking if the node is in obstacle
            # if not in obstacle, then only append in unvisited
            if node_obj.x <= min(ox) or node_obj.y <= min(oy) or node_obj.x >= max(ox) or node_obj.y >= max(oy): 
                continue
            
            c1 = check_boundary(node_obj.x,node_obj.y)
            c2 = check_rect(node_obj.x,node_obj.y)
            c3 = check_circle(node_obj.x,node_obj.y)
            c4 = check_ellipse(node_obj.x,node_obj.y)
            c5 = check_polygon(node_obj.x,node_obj.y)
            if c1 or c2 or c3 or c4 or c5:
                continue
  
            if node_key in visited.keys():    #=======================================
                #print("In visited")
                continue
            
            if node_key in temp_unvisited_keys.keys():
                #print("Yes")
                # cost and parent values update
                req_obj_tuple = temp_unvisited_keys[node_key]      #####ADDED
                #print(req_obj_tuple)
                if node_obj.gcost < req_obj_tuple[1]:
                    req_obj_list =list(req_obj_tuple)
                    req_obj_list[1] = node_obj.gcost
                    req_obj_list[0] = node_obj.gcost + h        # To change
                    req_obj_list[4] = node_obj.parent           # To change
                    req_obj_tuple = tuple(req_obj_list)
            
            else:                           #=======================================
                heapq.heappush(unvisited, node_obj.tuple())
                #heapq.heapify(unvisited)
                temp_unvisited_keys[node_key] = node_obj.tuple()   #####ADDED
                #plt.plot(node_obj.x,node_obj.y,"bx")
        
        visited[visited_index(current_node)] = current_node 
        j+=1
        #print("\nU :   ",unvisited)
        #print("\nV :   ",visited) 
        #print("\nNode_key:  ",temp_unvisited_keys)
    p_x, p_y = backtrack_optimal_path(goal_node, visited)
    return p_x, p_y
    
    
def backtrack_optimal_path(goal_node, visited):
    # generate lists of x and y coord of optimal path
    path_x = [goal_node.x]
    path_y = [goal_node.y]
    parent = goal_node.parent
    while parent != 0:
        temp = visited[parent]
        path_x.append(temp[2])
        path_y.append(temp[3])
        parent = temp[4]
    return path_x, path_y
    

if startp in zip(ox, oy) or goalp in zip(ox, oy):
    if startp in zip(ox, oy) :
        print("\nStart point is in obstacle space! Please enter valid point.")
    if goalp in zip(ox, oy) :
        print("\nGoal point is in obstacle space! Please enter valid point.")  
else:
    print("\nExploring the nodes...")
    req, req1 = astar(sx, sy, gx, gy, ox, oy)
    if len(req) == 1 or len(req1) == 1:
        print("\nThe path cannot be obtained because the start point or goal point is outside the map or inside closed obstacle region.")
    else:
        if flag_for_display:
            plt.plot(req, req1, "-g")
            plt.show() 
        #print("\n Explored path: ", req, req1)

end = time.time()
print("\nTime elapsed: ", abs(end-start), "sec \n")










