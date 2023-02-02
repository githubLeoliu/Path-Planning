import pygame

import common

class Window:
    def __init__(self, start, goal, map_dimensions, obs_dim, obs_num, algo_name):
        self.start = start
        self.goal = goal
        self.map_dimensions = map_dimensions
        self.map_height, self.map_width = self.map_dimensions

        # pygame window setting
        self.MapWindowName = algo_name
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.map_width, self.map_height))
        self.map.fill((255, 255, 255))
        self.node_rad = 2
        self.node_thickness = 0
        self.edge_thickness = 1

        self.obstacles = []
        self.obs_dim = obs_dim
        self.obs_num = obs_num

    def drawMap(self, obstacles):
        pygame.draw.circle(self.map, common.GREEN, self.start, self.node_rad + 5, 0)
        pygame.draw.circle(self.map, common.RED, self.goal, self.node_rad + 20, 1)
        self.drawObs(obstacles)

    def drawPath(self, path):
        for node in path:
            pygame.draw.circle(self.map, common.RED, node, self.node_rad + 3, 0)

    def drawObs(self, obstacles):
        obstaclesList = obstacles.copy()
        while obstaclesList:
            obstacle = obstaclesList.pop(0)
            pygame.draw.rect(self.map, common.GREY, obstacle)

    def drawVisited(self, nodelist):
        for node in nodelist:
            if node.parent:
                pygame.draw.line(self.map, common.GREEN,
                                 (node.parent.x, node.parent.y), (node.x, node.y),
                                 self.edge_thickness)

