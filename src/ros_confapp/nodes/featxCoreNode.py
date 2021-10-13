#!/usr/bin/env python3

import rospy
from ros_confapp.dsl.bindings import Bindings
from ros_confapp.dsl.engine import Engine
from ros_confapp.srv import *

class Config(Bindings):
    def __init__(self):
        Bindings.__init__(self)
        self.boundFeaturesQueue = Bindings.getAllBindings(self)

class RelayPing(Engine):
    def __init__(self):
        #rospy.init_node("ping")
        Engine.__init__(self)

    def runFeatureSource(self, commandArr):
        #read and loopthrough registry to get endpoint using active model and feature id mapping
        
        regList = self.readRegistry(commandArr[1])
        
        cmdStr = " ".join(commandArr)
        print(cmdStr)
        rospy.wait_for_service(regList[0])

        if regList != None:
            try:
                #print(regList)
                ping_src = rospy.ServiceProxy('dijkstra_node', 'dijkstra')
                resp = ping_src(cmdStr)
                return resp.path
            except rospy.ServiceException as e:
                rospy.loginfo("Source access failed")
                rospy.loginfo(e)


class Core(Config):
    def __init__(self):
        Config.__init__(self)
        rospy.init_node("core_feature_server")
        
    def serverListen(self):
        rospy.loginfo("Running Core")
        
        rospy.Service('get_cmd', processConfigAction, self.getExecutedCommand)
        
        rospy.spin()

    def getExecutedCommand(self, req):
        dslComms = req.cmd.split()
        print(dslComms)
        if dslComms[1].strip() in self.boundFeaturesQueue:
            if dslComms[0] == "load" or dslComms[0] == "unload":
                #check feature props against binding combos
                permission = self.checkBindingCombination(dslComms[1].strip().lower(), dslComms[0].strip().lower())
                if permission:
                    print('Loading feature....')
                    self.interpret(dslComms)
                else:
                    print('Feature could not be activated due to property constraints')
            elif dslComms[0] == "ping":
                relay = RelayPing()
                relay.runFeatureSource(dslComms)
            else:
                rospy.loginfo('DSL Command Executed: {}'.format(req.cmd))
        else:
            print("Sorry, abstract feature binding cannot be altered")

        return "Message Acknowledged"

    

def main():
   core = Core()
   core.serverListen()

if __name__ == '__main__':
    main()

