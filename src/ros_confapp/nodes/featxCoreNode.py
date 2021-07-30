#!/usr/bin/env python3

import rospy
from ros_confapp.dsl.bindings import Bindings
from ros_confapp.dsl.engine import Engine
from ros_confapp.srv import processConfigAction, processConfigActionRequest, processConfigActionResponse

class Config(Bindings):
    def __init__(self):
        Bindings.__init__(self)
        self.boundFeaturesQueue = Bindings.getAllBindings(self)

class coreFeatureNode(Config, Engine):
    def __init__(self):
        #TODO:Add dynamic imports
        #exec('from ros_confapp.featx.extracts.{}.root import RootBot'.format("zero"))
        Config.__init__(self)
        Engine.__init__(self)

    def getExecutedCommand(self, req):
        dslComms = req.cmd.split()
        if dslComms[1].strip() in self.boundFeaturesQueue:
            if dslComms[0] == "load" or dslComms[0] == "unload":
                #check feature props against binding combos
                permission = self.checkBindingCombination(dslComms[1].strip().lower(), dslComms[0].strip().lower())
                if permission:
                    print('Loading feature....')
                    self.interpret(dslComms)
                else:
                    print('Feature could not be activated due to property constraints')
            elif dslComms[0] == "load" == "ping":
                self.runFeatureSource(dslComms[1])
            else:
                rospy.loginfo('DSL Command Executed: {}'.format(req.cmd))
        else:
            print("Sorry, abstract feature binding cannot be altered")

        return "Message Acknowledged"

    def runFeatureSource(self, featureID):
        pass

def main():
    #fill queue with selections
    config = Config()
    
    rospy.init_node("core_feature_server")
    rospy.loginfo("Listening to DSL commands")
    fcore = coreFeatureNode()
    rospy.Service('get_cmd', processConfigAction, fcore.getExecutedCommand)
    
    rospy.spin()

if __name__ == '__main__':
    main()

