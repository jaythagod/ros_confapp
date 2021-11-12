#!/usr/bin/env python
import os
import json
import rospy

basePath = "C:/catkin_ws/src/ros_confapp/src/ros_confapp"

def getEngineState():
        engineStore = basePath+'/dsl/engineState.json'
        dataFile = open(engineStore, 'r')
        engineState = json.loads(dataFile.read())
        dataFile.close()
        return engineState

def readProps():
        eng = getEngineState()
        for proj in eng['projects']:
            if proj['status'] == 1:
                activeproj = proj['name']
                propsfile = basePath+'/model/usr/'+activeproj+'/config.json'
                dataFile = open(propsfile, 'r')
                allprops = json.loads(dataFile.read())
                dataFile.close()
                return allprops
        return False

def dumpCurrentConfig():
        configDump = []
        propList = readProps()
        for prop in propList['properties']:
            if prop['props']['time'] == "Early" and prop['props']['status'] == True:
                configDump.append(prop['id'])     
        return configDump

#dump config
currentConfig = dumpCurrentConfig()
#set global param list
rospy.set_param('global_config_param', currentConfig)

if rospy.get_param('/global_config_param'):
    rospy.loginfo('Static feature set loaded successfully')
