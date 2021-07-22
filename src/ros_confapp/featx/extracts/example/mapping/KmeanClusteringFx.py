from ros_confapp.featx.extracts.mapping.Clusters import Clusters
import math
import matplotlib.pyplot as plt
import random

class KmeanCluster(Clusters):
    def __init__(self, x, y, n_label):
        Clusters.__init__(x, y, n_label)
        self.MAX_LOOP = 10
        self.DCOST_TH = 0.1
        self.show_animation = False

    def kmeans_clustering(self, rx, ry, nc):
        clusters = Clusters(rx, ry, nc)
        clusters.calc_centroid()

        pre_cost = float("inf")
        for loop in range(self.MAX_LOOP):
            print("loop:", loop)
            cost = clusters.update_clusters()
            clusters.calc_centroid()

            d_cost = abs(cost - pre_cost)
            if d_cost < self.DCOST_TH:
                break
            pre_cost = cost

        return clusters

    def calc_raw_data(self, cx, cy, n_points, rand_d):
        rx, ry = [], []

        for (icx, icy) in zip(cx, cy):
            for _ in range(n_points):
                rx.append(icx + rand_d * (random.random() - 0.5))
                ry.append(icy + rand_d * (random.random() - 0.5))

        return rx, ry


    def update_positions(self, cx, cy):
        # object moving parameters
        DX1 = 0.4
        DY1 = 0.5
        DX2 = -0.3
        DY2 = -0.5

        cx[0] += DX1
        cy[0] += DY1
        cx[1] += DX2
        cy[1] += DY2

        return cx, cy

    def getClusters(self):
        print(__file__ + " start!!")

        cx = [0.0, 8.0]
        cy = [0.0, 8.0]
        n_points = 10
        rand_d = 3.0
        n_cluster = 2
        sim_time = 15.0
        dt = 1.0
        time = 0.0

        while time <= sim_time:
            print("Time:", time)
            time += dt

            # objects moving simulation
            cx, cy = self.update_positions(cx, cy)
            raw_x, raw_y = self.calc_raw_data(cx, cy, n_points, rand_d)

            clusters = self.kmeans_clustering(raw_x, raw_y, n_cluster)

            # for animation
            if self.show_animation:  # pragma: no cover
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
                clusters.plot_cluster()
                plt.plot(cx, cy, "or")
                plt.xlim(-2.0, 10.0)
                plt.ylim(-2.0, 10.0)
                plt.pause(dt)

            print("Done calculating clusters")
