import cv2
import numpy as np

from path_planning import *
from path_planning.a_star_planner import AStarPlanner


class AStarImplementation(AStarPlanner):
    # TODO: implement your own version of preloop, step and postloop
    def preloop(self):
        # This is for illustrative purposes only, feel free to modify
        self.queue: list[PathNode] = []
        self.g: dict[PathNode, float] = {}
        self.h: dict[PathNode, float] = {}
        self.visited_nodes: set[PathNode] = set()
        self.visited_nodes.add(self.start_node)
        self.queue.append(self.start_node)
        self.g[self.start_node] = 0
        self.h[self.start_node] = calculate_node_distance(
            self.start_node, self.goal_node
        )

    def step(self):
        # ===== some given data/parameters =====
        self.start_node
        self.goal_node
        self.world_map  # bgr
        self.occupancy_map  # bool
        self.goal_threshold
        self.grid_size  # for sampling neighbor nodes
        # ==========
        # to sample neighbor nodes, use
        # self.get_neighbor_nodes(current_node)'
        current_node = self.queue.pop(0)
        self.visited_nodes.add(current_node)

        if calculate_node_distance(current_node, self.goal_node) <= self.goal_threshold:
            self.goal_node.parent = current_node
            self.is_done.set()  # only set this on termination

        neighbor_nodes = self.get_neighbor_nodes(current_node)
        for neighbor_node in neighbor_nodes:
            if neighbor_node in self.visited_nodes:
                continue
            g_old = self.g.get(neighbor_node, float('inf'))
            if g_old > self.g[current_node] + calculate_node_distance( ####### = is important
                current_node, neighbor_node
            ):
                self.g[neighbor_node] = self.g[current_node] + calculate_node_distance(
                    current_node, neighbor_node
                )
                neighbor_node.parent = current_node
            # self.g[neighbor_node] = min(self.g[current_node] + calculate_node_distance(
            #     current_node, neighbor_node
            # ), g_old)
            self.h[neighbor_node] = calculate_node_distance(
                neighbor_node, self.goal_node
            )

            neighbor_node.cost = self.g[neighbor_node] + self.h[neighbor_node]
            if neighbor_node not in self.queue:
                self.queue.append(neighbor_node)
        # Sort the queue based on the cost of the nodes
        self.queue = sorted(self.queue, key=lambda node: node.cost)
        
    def postloop(self):
        return (
            collect_path(self.goal_node),
            self.visited_nodes
            #set(),  # replace with set of visited nodes
        )
