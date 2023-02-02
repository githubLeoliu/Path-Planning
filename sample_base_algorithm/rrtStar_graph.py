import math
import numpy as np
import random
import pygame

from graph import Graph
from window import Window
import common

class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None

class RrtStar_Graph(Graph):

    def __init__(self, start, goal, map_dimensions, obs_dim, obs_num):
        super().__init__(start, goal, map_dimensions, obs_dim, obs_num)

        self.s_start = Node(start)
        self.s_goal = Node(goal)

        self.step_len = 10
        self.goal_sample_rate = 0.1
        self.search_radius = 300
        self.iter_max = 2000
        self.vertex = [self.s_start]
        self.path = []

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    @staticmethod
    def cost(node_p):
        node = node_p
        cost = 0.0

        while node.parent:
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent

        return cost

    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_goal):

        dist, theta = self.get_distance_and_angle(node_start, node_goal)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))

        node_new.parent = node_start

        return node_new

    def get_new_cost(self, node_start, node_end):
        dist, _ = self.get_distance_and_angle(node_start, node_end)

        return self.cost(node_start) + dist

    def choose_parent(self, node_new, neighbor_index):
        cost = [self.get_new_cost(self.vertex[i], node_new) for i in neighbor_index]

        cost_min_index = neighbor_index[int(np.argmin(cost))]
        node_new.parent = self.vertex[cost_min_index]

    def rewire(self, node_new, neighbor_index):

        for i in neighbor_index:
            node_neighbor = self.vertex[i]

            if self.cost(node_neighbor) > self.get_new_cost(node_new, node_neighbor):
                print('rewire success')
                print("original parent: " + str((node_new.parent.x, node_new.parent.y)))
                node_new.parent = node_neighbor
                print("rewired parent: " + str((node_new.parent.x, node_new.parent.y)))
                print('\n')



    def search_goal_parent(self):
        dist_list = [math.hypot(n.x - self.s_goal.x, n.y - self.s_goal.y) for n in self.vertex]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.step_len]

        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.cost(self.vertex[i]) for i in node_index
                         if not self.cross_obstacle(self.vertex[i].x, self.vertex[i].y, self.s_goal.x, self.s_goal.y)]
            return node_index[int(np.argmin(cost_list))]

        return len(self.vertex) - 1

    def generate_random_node(self, goal_sample_rate):

        if np.random.random() > goal_sample_rate:
            x = int(random.uniform(0, self.map_width))
            y = int(random.uniform(0, self.map_height))
            return Node((x, y))

        return self.s_goal

    def find_near_neighbor(self, node_new):
        n = len(self.vertex) + 1
        r = self.search_radius * math.sqrt((math.log(n) / n))
        # min(self.search_radius * math.sqrt((math.log(n) / n)), self.step_len)

        dist_table = [math.hypot(nd.x - node_new.x, nd.y - node_new.y) for nd in self.vertex]
        dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind] <= r and
                            not self.cross_obstacle(node_new.x, node_new.y, self.vertex[ind].x, self.vertex[ind].y)]

        return dist_table_index

    def extract_path(self, node_end):
        path = [[self.s_goal.x, self.s_goal.y]]
        node = node_end

        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def is_free(self, n):
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rectang = obs.pop(0)
            if rectang.collidepoint(n.x, n.y):
                return False
        return True

    def plan(self):
        for k in range(self.iter_max):
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if self.is_free(node_new):
                if node_new and not self.cross_obstacle(node_near.x, node_near.y, node_new.x, node_new.y):
                    neighbor_index = self.find_near_neighbor(node_new)
                    self.vertex.append(node_new)

                    if neighbor_index:
                        self.choose_parent(node_new, neighbor_index)
                        self.rewire(node_new, neighbor_index)

        index = self.search_goal_parent()
        self.path = self.extract_path(self.vertex[index])

        return self.vertex, self.path

def main():
    dimension = (600, 1000)
    start = (50, 50)
    goal = (510, 510)
    obs_dim = 30
    obs_num = 50

    pygame.init()
    window = Window(start, goal, dimension, obs_dim, obs_num, 'RRT* path planning')
    graph = RrtStar_Graph(start, goal, dimension, obs_dim, obs_num)

    obstacles = graph.make_obstacle()

    window.drawMap(obstacles)
    pygame.display.update()

    node_list, path = graph.plan()

    for node in node_list:
        if node.parent:
            pygame.draw.line(window.map, common.GREEN,
                             (node.parent.x, node.parent.y),
                             (node.x, node.y),
                             window.edge_thickness)
        pygame.display.update()

    window.drawPath(path)

    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)

if __name__ == '__main__':
    main()