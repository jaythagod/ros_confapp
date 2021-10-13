from ros_confapp.featx.extracts.dijkstra.dijkstra import Dijkstra

import math

show_animation = False

class RunDijkstra():
    def __init__(self, startx, starty, goalx, goaly) :
        print("  |  ")
        print("Running Dijkstra...")
        self.sx = startx
        self.sy = starty
        self.gx = goalx
        self.gy = goaly

    def run(self):
        print("\nStart!!")

        # start and goal position
        grid_size = 2.0  # [m]
        robot_radius = 1.0  # [m]

        # set obstacle positions
        ox, oy = [], []
        for i in range(-10, 60):
            ox.append(i)
            oy.append(-10.0)
        for i in range(-10, 60):
            ox.append(60.0)
            oy.append(i)
        for i in range(-10, 61):
            ox.append(i)
            oy.append(60.0)
        for i in range(-10, 61):
            ox.append(-10.0)
            oy.append(i)
        for i in range(-10, 40):
            ox.append(20.0)
            oy.append(i)
        for i in range(0, 40):
            ox.append(40.0)
            oy.append(60.0 - i)

        if show_animation:  # pragma: no cover
            plt.plot(ox, oy, ".k")
            plt.plot(self.sx, self.sy, "og")
            plt.plot(self.gx, self.gy, "xb")
            plt.grid(True)
            plt.axis("equal")
        
        dijkstra = Dijkstra(ox, oy, grid_size, robot_radius)
        rx, ry = dijkstra.planning(self.sx, self.sy, self.gx, self.gy)

        return [rx,ry]