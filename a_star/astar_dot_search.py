#!/usr/bin/env python3

import networkx as nx
import matplotlib.pyplot as plt
import time
import math
import numpy as np
from operator import attrgetter

# variable definition
dot_folder = ''
dot_path = '' + dot_folder
dot_name = ''
image_folder = ''
image_name = ''
image_size = 480.0
dot_size = 140.0
image_size_res = image_size/(dot_size-1)*dot_size
patch_resolution = image_size_res/dot_size

# read dot graph
G = nx.drawing.nx_agraph.read_dot(dot_path + dot_name)

class Node():
    # Node class: used for astar search
    def __init__(self, parent= None, position = None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0
    def weight(self, neighbor):
        # returns the weight between the current and neighbor
        weight = G[str(self.position)][str(neighbor)]['probability']
        return weight

def create_neighbor_list(node):
    # create a list of all neighbors around the specified node
    neighbor_list = G.neighbors(str(node))
    neighbor_list = list(map(int, neighbor_list))
    neighbor_list = sorted(neighbor_list)
    return neighbor_list

def node_to_pixel(node):
    # caclulate the pixel coordinates of a node
    row, column = get_index(node)
    x = (column+1)*patch_resolution - patch_resolution/2.0
    y = (row+1)*patch_resolution - patch_resolution/2.0
    return x, y

def pixel_to_node(x, y):
    # this function takes a pixel coordinate and outputs the corresponding node
    # creates a numpy array filled with node numbers
    node_grid=np.zeros((140,140))
    counter = 0
    for row in range(140):
        for column in range(140):
            node_grid[row][column] = counter
            counter += 1
    column = int(x/patch_resolution)
    row = int(y/patch_resolution)
    node = int(node_grid[row][column])
    # outputs node as str() because that's what the search algorithms accept
    node = str(node)
    return node

def get_index(node):
    # gets the index of a specified node (eg. row and column)
    # accepts node in float(), returns float()
    row = node // dot_size
    column = node % dot_size
    return row, column

def dist_to_goal(node, goal):
    # accepts node and goal in float()
    # returns the pythagoreams distance
    x, y = get_index(node)
    x_goal, y_goal = get_index(goal)
    distance = math.sqrt(pow((x_goal - x), 2) + pow((y_goal - y), 2))
    return distance

def plot_path(path, path_name):
    # plot one scatter plots on top of the hm.png
    # initiate x, y lists
    x_list = []
    y_list = []
    # read the heightmap.png
    img = plt.imread(heightmaps_folder + heightmap_name + ".png")
    fig, ax = plt.subplots()
    #iterate over the path
    for i in path:
        x, y = node_to_pixel(i) 
        x_list.append(x)
        y_list.append(y)
    #plot the path on top of the heightmap image
    ax.scatter(x_list, y_list, s = 1, label=path_name)
    im = ax.imshow(img)
    plt.legend(loc='lower right')
    plt.show()

def a_star(start, goal):
    # astar algorithm: accepts start and end node in str()
    open_list = [] # list of Nodes()
    closed_list = []
    path = []
    added = 0
    count = -1 # index variables

    # Create start and end node
    start_node = Node(None, int(start))
    start_node.g = 0
    start_node.h = dist_to_goal(float(start_node.position), float(goal))
    start_node.f = start_node.g + start_node.h
    end_node = Node(None, int(goal))
    end_node.g = end_node.h = end_node.f = 0

    # add the start_node to the open list
    open_list.append(start_node)

    # while the open list is not empty
    while len(open_list) > 0:
        # take the first node within the open list
        current_node = open_list[0]
        current_index = 0
        # find the node with the lowest f value in the open list to set as the current node
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
        # remove the current node from the open list, and add in to the closed list
        open_list.pop(current_index)
        closed_list.append(current_node)
        # Found the end node
        if current_node.position == end_node.position:
            path.append(int(current_node.position))
            # look up the parent of every node and trace backwards to the start
            while current_node.parent is not None:
                for nodes in closed_list:
                    if nodes.position == current_node.parent:
                        current_node = nodes
                        path.append(int(current_node.position))
            path = sorted(path)            
            break
        # create a list of neighbors
        neighbor_list = create_neighbor_list(current_node.position)
        # create a list of neighbor nodes
        for neighbor in neighbor_list:
            neighbor_node = Node(None, neighbor)
            # find the cost of arriving: (1-probability)
            g_cost = (1-float(current_node.weight(neighbor_node.position))) * 10
            neighbor_node.g = float(current_node.g) + g_cost
            neighbor_node.h = dist_to_goal(float(neighbor_node.position), float(goal))
            neighbor_node.f = neighbor_node.g + neighbor_node.h
            # check if neighbor_node exist in open_list
            for nodes in open_list:
                count += 1
                if nodes.position == neighbor_node.position:
                    # if there is a cheaper arrival g_cost, replace with the new cost and parent
                    if nodes.g > neighbor_node.g:
                        nodes.g = neighbor_node.g
                        nodes.h = dist_to_goal(float(nodes.position), float(goal))
                        nodes.f = nodes.g + nodes.h
                        nodes.parent = current_node.position
                        open_list[count] = nodes
                    added = 1
            # check if neighbor_node exist in closed_list
            for nodes in closed_list:
                if nodes.position == neighbor_node.position:
                    added = 1
            # if the neighbor is not on both open and closed list, add it to the open list
            if added != 1:
                neighbor_node.parent = current_node.position
                open_list.append(neighbor_node)
            added = 0
            count = -1
    return path

start_node = pixel_to_node(87, 393)
goal_node = pixel_to_node(350, 97)

the_path = a_star(start_node, goal_node)
plot_path(the_path, 'astar')
