
import cv2
import numpy as np

from path_planning import *
from path_planning.rrt_star_planner import RRTStarPlanner


class RRTStarImplementation(RRTStarPlanner):
    # TODO: implement your own version of preloop, step and postloop
    def preloop(self):
        # This is for illustrative purposes only, feel free to modify
        self.visited_nodes:set[PathNode] = set()
        self.visited_nodes.add(self.start_node)

    def step(self):
        # ===== some given data/parameters =====
        self.start_node
        self.goal_node
        self.world_map # bgr
        self.occupancy_map # bool
        self.goal_threshold
        self.step_size # for steer
        self.search_radius # for reparent/rewire
        # ==========
        random_node = self.sample_random_node() # must use this to 
                                                # sample new node
        min_distance = float('inf')   
        near_nodes = set()
        nearest_node = list(self.visited_nodes)[0]   

        for node in self.visited_nodes:
            if calculate_node_distance(node, random_node) < min_distance:
                min_distance = calculate_node_distance(node, random_node)
                nearest_node = node
        ## Steer
        if calculate_node_distance(nearest_node, random_node) <= self.step_size:
            new_node = random_node 
        else:
                direction = np.array([random_node.coordinates.x - nearest_node.coordinates.x, random_node.coordinates.y - nearest_node.coordinates.y])
                direction = direction / np.linalg.norm(direction) * self.step_size
                new_x = nearest_node.coordinates.x + direction[0]
                new_y = nearest_node.coordinates.y + direction[1]
                new_node = PathNode(coordinates=PixelCoordinates(x=new_x, y=new_y))
        if not check_collision_free(self.occupancy_map, nearest_node, new_node):
            return
        # find near nodes for reparenting and rewiring
        for node in self.visited_nodes:
            if calculate_node_distance(node, new_node) < self.search_radius:
                    near_nodes.add(node)
        new_node.parent = nearest_node
        new_node.cost = calculate_node_distance(nearest_node, new_node) + nearest_node.cost

        # New Node is BORN
        self.visited_nodes.add(new_node)

        # RE-Parenting
        cost_min = float('inf')
        for near_node in near_nodes:
            dist = calculate_node_distance(near_node, new_node)
            if  dist + near_node.cost < cost_min and check_collision_free(self.occupancy_map, near_node, new_node):
                new_node.parent = near_node
                cost_min = dist + near_node.cost
                new_node.cost = cost_min

        # RE-Wiring
        for near_node in near_nodes:
            dist = calculate_node_distance(near_node, new_node)
            if dist + new_node.cost < near_node.cost and check_collision_free(self.occupancy_map, near_node, new_node):
                near_node.parent = new_node
                near_node.cost = dist + new_node.cost
                # self.update_children_cost(near_node) # NewFeature

        # check if new node can connect to goal node
        if calculate_node_distance(new_node, self.goal_node) < self.goal_threshold:
            self.goal_node.parent = new_node
            self.goal_node.cost = calculate_node_distance(new_node, self.goal_node) + new_node.cost

            self.is_done.set() # only set this on termination

    def update_children_cost(self, parent_node: PathNode): # NewFeature
        """
        當 parent_node 的成本下降時，
        更新所有以它為 parent 的子節點的成本。
        """
        for node in self.visited_nodes:
            if node.parent == parent_node:
                node.cost = parent_node.cost + calculate_node_distance(node, parent_node)
                self.update_children_cost(node)
            
    def postloop(self):
        return (
            collect_path(self.goal_node), 
            #set() # replace with set of visited nodes
            self.visited_nodes
        )