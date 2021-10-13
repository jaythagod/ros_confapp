#from ..dijkstra.runDijkstra import RunDijkstra
from ros_confapp.featx.extracts.base.dijkstra.runDijkstra import RunDijkstra

class PathPlanning(RunDijkstra):
    def __init__(self):
        print("  |  ")
        print("Path Planning")
        RunDijkstra.__init__(self)