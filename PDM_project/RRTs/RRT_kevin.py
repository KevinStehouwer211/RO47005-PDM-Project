import numpy as np
import random
import math

# RRT route planning class
class RRTKevin:
    
    # Initialize class parameters 
    def __init__(self, start, goal, rand_area, obstacle_list, expand_dis = .5, max_iter = 500, goal_sample = 10):
        self.start = Node(start[0],start[1],start[2])
        self.goal = Node(goal[0],goal[1],goal[2])
        self.rand_min = rand_area[0]
        self.rand_max = rand_area[1]
        self.obstacles = obstacle_list
        self.expand_dis = expand_dis
        self.max_iter = max_iter
        self.goal_sample = goal_sample
        self.node_list = []
        self.counter = 0
    

    # Main function for planning the route using RRT
    def plan(self):

        # Create a list to store nodes, containing the starting point 
        self.node_list = [self.start]
        path_length = float("inf")
        shortest_path_length = float("inf")
        shortest_path = []

        # Iterate for a certain amount to create a path with RRT
        for iter in range(self.max_iter):
            random_node = self.create_random_node()
            closest_node = self.compute_closest_node(random_node, self.node_list)
            new_node = self.steer(closest_node,random_node)

            collision = self.check_collision(new_node, closest_node, self.obstacles)
            distance_to_goal = self.compute_distance(new_node,self.goal)

            if not collision:
                self.node_list.append(new_node)
                new_node.parent = closest_node
                
                if distance_to_goal <= self.expand_dis:
                    collisision_to_goal = self.check_collision(new_node,self.goal, self.obstacles)

                    if not collisision_to_goal:
                        goal_node = Node(self.goal.x,self.goal.y,self.goal.z)
                        goal_node.parent = new_node
                        self.node_list.append(goal_node)
                        path, path_length = self.define_path(self.node_list, goal_node)
                    
                        if path_length < shortest_path_length:
                            shortest_path_length = path_length
                            shortest_path = path

        return shortest_path if len(shortest_path) else None
    

    # Function for generating random nodes
    def create_random_node(self):

        if self.counter < 10:
            # Create lists of possible values for x, y and z within the search area
            rand_area_x = np.arange(self.rand_min[0], self.rand_max[0], 0.01)
            rand_area_y = np.arange(self.rand_min[1], self.rand_max[1], 0.01)
            rand_area_z = np.arange(self.rand_min[2], self.rand_max[2], 0.01)

            # Generate random samples
            random_x = np.random.choice(rand_area_x)
            random_y = np.random.choice(rand_area_y)
            random_z = np.random.choice(rand_area_z)

            # Create a node containing the random coordinates
            random_node = Node(random_x, random_y, random_z)
            self.counter += 1

        elif self.counter == 10:
            random_node = Node(self.goal.x,self.goal.y,self.goal.z)
            self.counter = 0

        return random_node

    # Function to compute the closest node of the tree to the random new node
    def compute_closest_node(self, random_node, node_list):
        closest_distance = float("inf")
        closest_node = None

        for node in node_list:
            distance = self.compute_distance(random_node,node)
            if distance < closest_distance:
                closest_distance = distance
                closest_node = node

        return closest_node

    # Function to compute the distance between two nodes
    def compute_distance(self, start, end):
        return np.sqrt((start.x-end.x)**2+(start.y-end.y)**2+(start.z-end.z)**2)
    
    def steer(self, from_node, to_node):
        distance = self.compute_distance(from_node, to_node)
        if distance <= self.expand_dis:
            return Node(to_node.x, to_node.y, to_node.z)

        ratio = self.expand_dis / distance
        x = from_node.x + ratio * (to_node.x - from_node.x)
        y = from_node.y + ratio * (to_node.y - from_node.y)
        z = from_node.z + ratio * (to_node.z - from_node.z)
        return Node(x, y, z)

    
    # Function to check for collision between node or path and obstacles
    def check_collision(self, random_node, closest_node, obstacles):

        # Define collision variable and compute path between closest- and random node
        collision = False
        path = self.interpolate_path(random_node,closest_node)

        for obstacle in obstacles:
            x_min = obstacle[0] - obstacle[3]
            x_max = obstacle[0] + obstacle[3]
            y_min = obstacle[1] - obstacle[4]
            y_max = obstacle[1] + obstacle[4]
            z_min = obstacle[2] - obstacle[5]
            z_max = obstacle[2] + obstacle[5]

            if x_min < random_node.x < x_max and y_min < random_node.y < y_max and z_min < random_node.z < z_max:
                collision = True
                return collision
            
            for step in path:
                if x_min < step[0] < x_max and y_min < step[1] < y_max and z_min < step[2] < z_max:
                    collision = True
                    return collision
            
        return collision
    
    # Function to create a path between two nodes with a certain step size
    def interpolate_path(self, start, end, step = 0.1):

        start = np.array([start.x, start.y, start.z], dtype=float)
        end = np.array([end.x, end.y, end.z], dtype=float)

        vec = end - start
        dist = np.linalg.norm(vec)

        if dist == 0:
            return start.reshape(1, 3)

        n = int(np.ceil(dist / step)) + 1
        t = np.linspace(0.0, 1.0, n)

        path = start[None, :] + t[:, None] * vec[None, :]
        return path


    # Function to compute the final path in x, y and z coordinates
    def define_path(self, node_list, goal_node):

        path = [goal_node]
        path_distance = 0
        node = goal_node

        while node is not None:
            if node == self.start:
                break

            parent = node.parent
            if parent is None:
                break

            path.insert(0,node.parent)
            path_distance += self.compute_distance(node,node.parent)
            node = parent

        path = np.array([[node.x,node.y,node.z] for node in path])

        return path, path_distance


# Node data class
class Node:
    # Initialize class parameters
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None


