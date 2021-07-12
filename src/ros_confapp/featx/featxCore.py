from ros_confapp.featx.extracts.Control import Control
from ros_confapp.featx.extracts.Navigation import Navigation

class FeatxCore(Control, Navigation):
    def __init__(self):
        self.id = "root_bot"

    def loadControl(self):
        Control.__init__(self)
        self.getControl()

    def loadNavigation(self):
        Navigation.__init__(self)
        self.getNavigation()
