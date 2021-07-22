from ros_confapp.featx.extracts.base.dijkstra.dijkstra import Dijkstra

import math

show_animation = False

class RunDijkstra():
    def __init__(self) :
        print("  |  ")
        print("Running Dijkstra...")

    def run(self):
        print("\nStart!!")

        # start and goal position
        sx = -5.0  # [m]
        sy = -5.0  # [m]
        gx = 50.0  # [m]
        gy = 50.0  # [m]
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
            plt.plot(sx, sy, "og")
            plt.plot(gx, gy, "xb")
            plt.grid(True)
            plt.axis("equal")
        
        dijkstra = Dijkstra(ox, oy, grid_size, robot_radius)
        rx, ry = dijkstra.planning(sx, sy, gx, gy)

        print(f'[rx]={rx}')
        print(f'\n[ry]={ry}')