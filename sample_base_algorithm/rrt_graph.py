from graph import Graph
import random
import math
import pygame
import time

from window import Window
import common


class RRT_Graph(Graph):
    def __init__(self, start, goal, map_dimensions, obs_dim, obs_num):
        super().__init__(start, goal, map_dimensions, obs_dim, obs_num)

        # init start
        (x, y) = start
        self.x = []
        self.y = []
        self.parent = []

        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)

    def sample_envir(self):
        x=int(random.uniform(0, self.map_width))
        y=int(random.uniform(0, self.map_height))
        return x, y

    def nearest(self, n):
        dmin = self.distance(0, n)
        nnear = 0
        for i in range(n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear

    def bias(self, ngoal):
        n = self.number_of_node()
        self.add_node(ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear, n)
        self.connect(nnear, n)
        return self.x, self.y, self.parent

    def expand(self):
        n = self.number_of_node()
        x, y = self.sample_envir()
        self.add_node(x, y)
        if self.is_free():
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            self.connect(xnearest, n)
        return self.x, self.y, self.parent

    def step(self, nnear, nrand, dmax=35):
        d = self.distance(nnear, nrand)
        if d > dmax:
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])
            (px, py) = (xrand - xnear, yrand - ynear)
            theta = math.atan2(py, px)
            (x, y) = (int(xnear + dmax * math.cos(theta)),
                      int(ynear + dmax * math.sin(theta)))
            self.remove_node(nrand)

            # if new node reach the target area
            if abs(x - self.goal[0]) < dmax and abs(y - self.goal[1]) < dmax:
                self.add_node(x, y)
                self.goal_state = nrand
                self.goalFlag = True
            else:
                self.add_node(x, y)

    def add_node(self, x, y):
        self.x.append(x)
        self.y.append(y)

    def remove_node(self, n):
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self, parent, child):
        self.parent.insert(child, parent)

    def remove_edge(self, n):
        self.parent.pop(n)

    def number_of_node(self):
        return len(self.x)

    def distance(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        px = (float(x1) - float(x2)) ** 2
        py = (float(y1) - float(y2)) ** 2
        return (px + py) ** (0.5)

    def is_free(self):
        ''' determine if node is in free space '''
        n = self.number_of_node() - 1
        (x, y) = (self.x[n], self.y[n])
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rectang = obs.pop(0)
            if rectang.collidepoint(x, y):
                self.remove_node(n)
                return False
        return True

    def connect(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        if self.cross_obstacle(x1, y1, x2, y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1, n2)
            return True

    def path_to_goal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goal_state)
            newpos = self.parent[self.goal_state]
            while newpos != 0:
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goalFlag

    def get_path_coordinates(self):
        pathCoords = []
        for node in self.path:
            x, y = (self.x[node], self.y[node])
            pathCoords.append((x, y))
        return pathCoords

def main():
    dimension = (600, 1000)
    start = (50, 50)
    goal = (510, 510)
    obs_dim = 30
    obs_num = 50
    iteration = 0

    pygame.init()
    window = Window(start, goal, dimension, obs_dim, obs_num, 'RRT path planning')
    graph = RRT_Graph(start, goal, dimension, obs_dim, obs_num)

    obstacles = graph.make_obstacle()

    window.drawMap(obstacles)

    while not graph.path_to_goal():
        time.sleep(0.1)

        if iteration % 10 == 0:
            X, Y, Parent = graph.bias(goal)
            pygame.draw.circle(window.map, common.GREY, (X[-1], Y[-1]), window.node_rad + 2, 0)
            pygame.draw.line(window.map, common.BLUE, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
                             window.edge_thickness)
        else:
            X, Y, Parent = graph.expand()
            pygame.draw.circle(window.map, common.GREY, (X[-1], Y[-1]), window.node_rad + 2, 0)
            pygame.draw.line(window.map, common.BLUE, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
                             window.edge_thickness)

        # if iteration % 5 == 0:
        pygame.display.update()

        iteration += 1

    window.drawPath(graph.get_path_coordinates())
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)

if __name__ == '__main__':
    main()