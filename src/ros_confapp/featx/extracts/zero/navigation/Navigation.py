#from ..path_planning.PathPlanning import PathPlanning
from ros_confapp.featx.extracts.base.path_planning.PathPlanning import PathPlanning

class Navigation(PathPlanning):
    def __init__(self):
        print("  |  ")
        print("Navigation")
        PathPlanning.__init__(self)