import random
import pygame

class Graph():
    def __init__(self, start, goal, map_dimensions, obs_dim, obs_num):
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.map_height, self.map_width = map_dimensions

        # obstacles
        self.obstacles = []
        self.obs_dim = obs_dim
        self.obs_num = obs_num

        # path
        self.goal_state = None
        self.path = []

    def make_random_rect(self):
        uppercorner_x = int(random.uniform(0, self.map_width - self.obs_dim))
        uppercorner_y = int(random.uniform(0, self.map_height - self.obs_dim))

        return (uppercorner_x, uppercorner_y)

    def make_obstacle(self):
        obs = []

        for i in range(self.obs_num):
            rectang = None
            startgoalcol = True
            while startgoalcol:
                upper = self.make_random_rect()
                rectang = pygame.Rect(upper, (self.obs_dim, self.obs_dim))
                if rectang.collidepoint(self.start) or rectang.collidepoint(self.goal):
                    startgoalcol = True
                else:
                    startgoalcol = False
            obs.append(rectang)
        self.obstacles = obs.copy()
        return obs

    def cross_obstacle(self, x1, y1, x2, y2):
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rectang = obs.pop(0)
            for i in range(0, 101):
                u = i/100
                x = x1*u + x2*(1-u)
                y = y1*u + y2*(1-u)
                if rectang.collidepoint(x, y):
                    return True
        return False


