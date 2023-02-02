import math, heapq, pickle

import plotting, env

class JPS:
    def __init__(self, s_start, s_goal):
        self.s_start = s_start
        self.s_goal = s_goal

        self.Env = env.Env()  # class Env

        self.obs = self.Env.obs  # position of obstacles

        self.OPEN = []  # priority queue / OPEN set
        self.CLOSED = []  # CLOSED set / VISITED order
        self.PARENT = dict()  # recorded parent
        self.g = dict()  # cost to come
        self.f = dict()

    def heuristic(self, s):
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """
        goal = self.s_goal  # goal node

        return math.hypot(goal[0] - s[0], goal[1] - s[1])

    def blocked(self, cX, cY, dX, dY):
        if cX + dX < 0 or cX + dX >= self.Env.x_range:
            return True
        if cY + dY < 0 or cY + dY >= self.Env.y_range:
            return True

        if dX != 0 and dY != 0:
            if (cX + dX, cY) in self.obs and (cX, cY + dY) in self.obs:
                return True
            if (cX + dX, cY + dY) in self.obs:
                return True
        else:
            if dX != 0:
                if (cX + dX, cY) in self.obs:
                    return True
            else:
                if (cX, cY + dY) in self.obs:
                    return True
        return False

    def dblock(self, cX, cY, dX, dY):
        if (cX - dX, cY) in self.obs and (cX, cY - dY) in self.obs:
            return True
        else:
            return False

    def direction(self, cX, cY, pX, pY):
        dX = int(math.copysign(1, cX - pX))
        dY = int(math.copysign(1, cY - pY))
        if cX - pX == 0:
            dX = 0
        if cY - pY == 0:
            dY = 0
        return (dX, dY)

    def nodeNeighbours(self, cX, cY, parent):
        neighbours = []
        if type(parent) != tuple:
            for i, j in [
                (-1, 0),
                (0, -1),
                (1, 0),
                (0, 1),
                (-1, -1),
                (-1, 1),
                (1, -1),
                (1, 1),
            ]:
                if not self.blocked(cX, cY, i, j):
                    neighbours.append((cX + i, cY + j))
            return neighbours


        dX, dY = self.direction(cX, cY, parent[0], parent[1])

        if dX != 0 and dY != 0:
            if not self.blocked(cX, cY, 0, dY):
                neighbours.append((cX, cY + dY))
            if not self.blocked(cX, cY, dX, 0):
                neighbours.append((cX + dX, cY))
            if (
                    not self.blocked(cX, cY, 0, dY)
                    or not self.blocked(cX, cY, dX, 0)
            ) and not self.blocked(cX, cY, dX, dY):
                neighbours.append((cX + dX, cY + dY))
            if self.blocked(cX, cY, -dX, 0) and not self.blocked(
                    cX, cY, 0, dY
            ):
                neighbours.append((cX - dX, cY + dY))
            if self.blocked(cX, cY, 0, -dY) and not self.blocked(
                    cX, cY, dX, 0
            ):
                neighbours.append((cX + dX, cY - dY))

        else:
            if dX == 0:
                if not self.blocked(cX, cY, dX, 0):
                    if not self.blocked(cX, cY, 0, dY):
                        neighbours.append((cX, cY + dY))
                    if self.blocked(cX, cY, 1, 0):
                        neighbours.append((cX + 1, cY + dY))
                    if self.blocked(cX, cY, -1, 0):
                        neighbours.append((cX - 1, cY + dY))

            else:
                if not self.blocked(cX, cY, dX, 0):
                    if not self.blocked(cX, cY, dX, 0):
                        neighbours.append((cX + dX, cY))
                    if self.blocked(cX, cY, 0, 1):
                        neighbours.append((cX + dX, cY + 1))
                    if self.blocked(cX, cY, 0, -1):
                        neighbours.append((cX + dX, cY - 1))
        return neighbours

    def jump(self, cX, cY, dX, dY, goal):
        nX = cX + dX
        nY = cY + dY
        if self.blocked(nX, nY, 0, 0):
            return None

        if (nX, nY) == goal:
            return (nX, nY)

        oX = nX
        oY = nY

        if dX != 0 and dY != 0:
            while True:
                if (
                        not self.blocked(oX, oY, -dX, dY)
                        and self.blocked(oX, oY, -dX, 0)
                        or not self.blocked(oX, oY, dX, -dY)
                        and self.blocked(oX, oY, 0, -dY)
                ):
                    return (oX, oY)

                if (
                        self.jump(oX, oY, dX, 0, goal) != None
                        or self.jump(oX, oY, 0, dY, goal) != None
                ):
                    return (oX, oY)

                oX += dX
                oY += dY

                if self.blocked(oX, oY, 0, 0):
                    return None

                if self.dblock(oX, oY, dX, dY):
                    return None

                if (oX, oY) == goal:
                    return (oX, oY)
        else:
            if dX != 0:
                while True:
                    if (
                            not self.blocked(oX, nY, dX, 1)
                            and self.blocked(oX, nY, 0, 1)
                            or not self.blocked(oX, nY, dX, -1)
                            and self.blocked(oX, nY, 0, -1)
                    ):
                        return (oX, nY)

                    oX += dX

                    if self.blocked(oX, nY, 0, 0):
                        return None

                    if (oX, nY) == goal:
                        return (oX, nY)

            else:
                while True:
                    if (
                            not self.blocked(nX, oY, 1, dY)
                            and self.blocked(nX, oY, 1, 0)
                            or not self.blocked(nX, oY, -1, dY)
                            and self.blocked(nX, oY, -1, 0)
                    ):
                        return (nX, oY)

                    oY += dY

                    if self.blocked(nX, oY, 0, 0):
                        return None

                    if (nX, oY) == goal:
                        return (nX, oY)

    def identifySuccessors(self, cX, cY, came_from, goal):
        successors = []
        neighbours = self.nodeNeighbours(cX, cY, came_from.get((cX, cY), 0))

        for cell in neighbours:
            dX = cell[0] - cX
            dY = cell[1] - cY

            jumpPoint = self.jump(cX, cY, dX, dY, goal)

            if jumpPoint != None:
                successors.append(jumpPoint)

        return successors

    def searching(self):
        self.g[self.s_start] = 0
        self.f[self.s_start] = self.heuristic(self.s_start)

        heapq.heappush(self.OPEN, (self.f[self.s_start], self.s_start))

        while self.OPEN:
            current = heapq.heappop(self.OPEN)[1]
            if current == self.s_goal:
                path = []
                while current in self.PARENT:
                    path.append(current)
                    current = self.PARENT[current]
                path.append(self.s_start)
                path = path[::-1]
                return path, self.CLOSED

            self.CLOSED.append(current)
            successors = self.identifySuccessors(current[0], current[1], self.PARENT, self.s_goal)

            for successor in successors:
                jumpPoint = successor

                if jumpPoint in self.CLOSED:
                    continue

                g_score = self.g[current] + self.dist(current, jumpPoint)

                if g_score < self.g.get(jumpPoint,0) or jumpPoint not in [j[1] for j in self.OPEN]:
                    self.PARENT[jumpPoint] = current
                    self.g[jumpPoint] = g_score
                    self.f[jumpPoint] = g_score + self.heuristic(jumpPoint)
                    heapq.heappush(self.OPEN, (self.f[jumpPoint], jumpPoint))
        return 0

    def dist(self, start, end):
        return math.hypot(start[0] - end[0], start[1] - end[1])

def main():
    s_start = (5, 5)
    s_goal = (45, 25)

    jps = JPS(s_start, s_goal)
    plot = plotting.Plotting(s_start, s_goal)

    path, visited = jps.searching()
    plot.animation(path, visited, "JPS")  # animation

    data_serie = {
        "map": {
            "x_range": jps.Env.x_range,
            "y_range": jps.Env.y_range,
            "obs": jps.Env.obs
        },
        "start": s_start,
        "goal": s_goal,
        "path": path
    }

    # write to pickle file
    pickle.dump(data_serie, open('../pickle_files/jps.pkl', 'wb'))

if __name__ == '__main__':
    main()