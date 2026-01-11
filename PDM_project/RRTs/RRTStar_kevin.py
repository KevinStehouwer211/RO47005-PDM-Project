import numpy as np
import random
import math

# RRT route planning class
class RRTStarKevin:
    
    # Initialize class parameters 
    def __init__(self, start, goal, rand_area, obstacle_list, expand_dis = .2, max_iter = 1000, goal_sample = 10):
        self.start = Node(start[0],start[1],start[2])
        self.goal = Node(goal[0],goal[1],goal[2])
        self.rand_min = rand_area[0]
        self.rand_max = rand_area[1]
        self.obstacles = obstacle_list
        self.expand_dis = expand_dis
        self.max_iter = max_iter
        self.goal_sample = goal_sample
        self.node_list = []

    # Main function for planning the route using RRT
    def plan(self):

        # Create a list to store nodes, containing the starting point 
        self.node_list = [self.start]
        lowest_cost = float("inf")
        shortest_path = []

        # Iterate for a certain amount to create a path with RRT
        for iter in range(self.max_iter):
            # Create random node within the rand_area
            random_node = self.create_random_node()
            closest_node = self.compute_closest_node(random_node)
            new_node = self.steer(closest_node,random_node)


            if self.compute_distance(new_node, closest_node) < 1e-6:
                continue

            collision = self.check_collision(new_node, closest_node)
            distance_to_goal = self.compute_distance(new_node,self.goal)

            if not collision:
                search_radius = self.current_radius()
                near_nodes = self.compute_near_nodes(new_node,self.node_list,search_radius)
                parent, cost = self.compute_best_neighbour(new_node, near_nodes)

                if parent is None:
                    parent = closest_node
                    cost = closest_node.cost + self.compute_distance(closest_node,new_node)

                new_node.parent = parent
                new_node.cost = cost
                self.node_list.append(new_node)

                self.rewire(new_node, near_nodes)
                
                if distance_to_goal <= self.expand_dis:
                    collisision_to_goal = self.check_collision(new_node,self.goal)

                    if not collisision_to_goal:
                        goal_node = Node(self.goal.x,self.goal.y,self.goal.z)

                        search_radius = self.current_radius()
                        goal_near_nodes = self.compute_near_nodes(goal_node,self.node_list,search_radius)
                        goal_parent, goal_cost = self.compute_best_neighbour(goal_node, goal_near_nodes)

                        if goal_parent is None:
                            goal_parent = new_node
                            goal_cost = new_node.cost + self.compute_distance(goal_node,new_node)

                        goal_node.parent = goal_parent
                        goal_node.cost = goal_cost

                        path = self.define_path(goal_node)
                    
                        if goal_cost < lowest_cost:
                            lowest_cost = goal_node.cost
                            shortest_path = path

        return shortest_path if len(shortest_path) else None
    

    # Function for generating random nodes
    def create_random_node(self):
        # 10% goal bias 
        if np.random.randint(0,100) < 10:
            return Node(self.goal.x, self.goal.y, self.goal.z)

        x = np.random.uniform(self.rand_min[0], self.rand_max[0])
        y = np.random.uniform(self.rand_min[1], self.rand_max[1])
        z = np.random.uniform(self.rand_min[2], self.rand_max[2])
        return Node(x, y, z)


    # Function to compute the closest node of the tree to the random new node
    def compute_closest_node(self, random_node):
        closest_distance = float("inf")
        closest_node = None

        # Loop over the node list and find the closest node
        for node in self.node_list:
            distance = self.compute_distance(random_node,node)
            if distance < closest_distance:
                closest_distance = distance
                closest_node = node

        return closest_node

    # Function to compute the distance between two nodes
    def compute_distance(self, start, end):
        return np.sqrt((start.x-end.x)**2+(start.y-end.y)**2+(start.z-end.z)**2)
    
    # Steering function from one node to the next
    def steer(self, from_node, to_node):

        # Compute the distance between the nodes and store the node when within the expand distance
        distance = self.compute_distance(from_node, to_node)
        if distance <= self.expand_dis:
            return Node(to_node.x, to_node.y, to_node.z)

        # Compute the distance ratio and move in the direction of the new node
        ratio = self.expand_dis / distance
        x = from_node.x + ratio * (to_node.x - from_node.x)
        y = from_node.y + ratio * (to_node.y - from_node.y)
        z = from_node.z + ratio * (to_node.z - from_node.z)
        return Node(x, y, z)

    
    # Function to check for collision between node or path and obstacles
    def check_collision(self, random_node, closest_node):

        # Define collision variable and compute path between closest- and random node
        collision = False
        step = self.expand_dis/5
        path = self.interpolate_path(random_node,closest_node,step)

        # Loop over all obstacles
        for obstacle in self.obstacles:
            hx, hy, hz = obstacle[3], obstacle[4], obstacle[5]
            x_min, x_max = obstacle[0] - hx, obstacle[0] + hx
            y_min, y_max = obstacle[1] - hy, obstacle[1] + hy
            z_min, z_max = obstacle[2] - hz, obstacle[2] + hz        
            
            # Compute whether the path between two nodes goes through an obstacle
            for step in path:
                if x_min < step[0] < x_max and y_min < step[1] < y_max and z_min < step[2] < z_max:
                    collision = True
                    return collision
                    
        return collision
    
    # Function to create a path between two nodes with a certain step size
    def interpolate_path(self, start, end, step):

        start = np.array([start.x, start.y, start.z], dtype=float)
        end = np.array([end.x, end.y, end.z], dtype=float)

        # Compute the vector between two nodes and its length
        vec = end - start
        dist = np.linalg.norm(vec)

        if dist == 0:
            return start.reshape(1, 3)

        # Determine the number of steps
        n = int(np.ceil(dist / step)) + 1
        t = np.linspace(0.0, 1.0, n)

        # Compute the path between two nodes
        path = start[None, :] + t[:, None] * vec[None, :]
        return path
    

    # Function for the search_radius veriable
    def current_radius(self):
        n = len(self.node_list) + 1
        d = 3
        gamma = 2.0  # tune (start 2–5)
        r = gamma * (math.log(n) / n) ** (1.0 / d)
        return max(r, self.expand_dis)  # don’t go below step size


    # Function for finding the nodes within the search radius of the new node
    def compute_near_nodes(self, new_node, node_list, search_radius):

        near_nodes = []
        r = search_radius

        # Loop over the node list
        for node in node_list:
            distance = self.compute_distance(node,new_node)

            # Store nodes within the search radius
            if distance <= r:
                near_nodes.append(node)

        return near_nodes
    
    # Function for computing the neighbour containing the total lowest cost
    def compute_best_neighbour(self, new_node, near_nodes):

        lowest_cost = float("inf")
        best_neighbour = None

        # Loop over all near nodes and compute the cost
        for node in near_nodes:
            collision = self.check_collision(new_node,node)

            if not collision:
                cost = node.cost
                add_cost = self.compute_distance(new_node, node)
                total_cost = cost + add_cost

                if total_cost < lowest_cost:
                    lowest_cost = total_cost
                    best_neighbour = node
        
        return best_neighbour, lowest_cost
    
    # Function for rewiring the tree if theres a path containing a lower cost
    def rewire(self,new_node, near_nodes):
        
        for node in near_nodes:

            new_cost = new_node.cost + self.compute_distance(new_node, node)

            if new_cost > node.cost:
                continue

            if self.check_collision(node, new_node):
                continue

            node.parent = new_node
            node.cost = new_cost


    # Function to compute the final path in x, y and z coordinates
    def define_path(self, goal_node):

        path = [goal_node]
        node = goal_node
        visited = set()

        # Loop over all parents 
        while node is not None:
            if id(node) in visited:
                raise RuntimeError("Cycle detected in parent pointers")
            visited.add(id(node))

            if node == self.start:
                break

            parent = node.parent
            if parent is None:
                break

            # Store all parents in reversed order and compute the path distance
            path.insert(0,node.parent)
            node = parent

        # Compute the path
        path = np.array([[node.x,node.y,node.z] for node in path])

        return path


# Node data class
class Node:
    # Initialize class parameters
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None
        self.cost = 0.0     # Cost from start to node