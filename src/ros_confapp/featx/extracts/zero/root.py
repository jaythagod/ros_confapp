from ros_confapp.featx.extracts.base.navigation.Navigation import Navigation

class RootBot(Navigation):
    def __init__(self):
        print("Root Bot")
        Navigation.__init__(self)