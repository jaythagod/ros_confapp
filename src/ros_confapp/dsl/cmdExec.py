#!/usr/bin/env python
import rospy
import rosnode
import roslaunch
from std_msgs.msg import String
import os
import shutil
import uuid
from distutils.util import strtobool

from ros_confapp.dsl.documentation import Documentation
from ros_confapp.dsl.dslState import DslState
from ros_confapp.dsl.configurator import Configurator


#from dsl.config import Configuration

class CmdExec(Configurator, DslState, Documentation):
    def __init__(self):
        DslState.__init__(self)
        Documentation.__init__(self)
        Configurator.__init__(self)
        self._usrProjectsDir = os.path.dirname(os.path.abspath('../model/usr/base'))
        self._stringPath = ""
        self._traverseGuide = []
        self._lookupDataSet = []

    def ls(self):
        engState = self.getEngineState()
        for proj in engState['projects']:
            if proj['status'] == 1:
                print('|')
                print('|--'+ proj['name']+'**')
            else:
                print('|')
                print('|--'+ proj['name'])

    def remove(self, paramList):
        model = self.readModel()
        remCmdList = self.validateRemoveCommandStructure(paramList)

        if len(remCmdList[0]) > 0:
            confirmation = input(f' Confirm feature[{remCmdList[0][1]}] REMOVAL (y/n): ')

            if confirmation[0] == 'y' or confirmation[0]=='Y':
                exec("del model"+remCmdList[1])
                self.removalCleanup(remCmdList[0][1])
                self.saveModel(model)
                print(f'--Feature({remCmdList[0][1]}) removed')
            else:
                print(f'--Feature({remCmdList[0][1]}) removal CANCELLED')

    def removalCleanup(self, featureId):
        self.removeIndex(featureId)
        self.removeProps(featureId)


    def removeIndex(self, featureId):
        counter = 0
        deleteIndex = None
        indexList = self.readIndexLookup()

        for index in indexList['mappings']:
            if index['child'] == featureId:
                deleteIndex = counter
            counter += 1

        del indexList['mappings'][deleteIndex]
        self.saveIndexLookup(indexList)
        print(f'\t--Feature({featureId}) index reference removed')


    def removeProps(self, featureId):
        counter = 0
        deleteIndex = None
        propsList = self.readProps()

        for prop in propsList['properties']:
            if prop['id'] == featureId:
                deleteIndex = counter
            counter += 1

        del propsList['properties'][deleteIndex]
        self.saveProps(propsList)
        print(f'\t--Feature({featureId}) properties reference removed')


    def validateRemoveCommandStructure(self, dslCommand):
        self.resetPath()
        removeRule = self.returnSingleLexRule("remove")
        if len(dslCommand) == removeRule['length']:
            self.buildPath(dslCommand[1])
            return [dslCommand, self._stringPath]
        else:
            self.printError(50)


    def add_feature(self, addPoint, objectAddition):
        self.executeFeatureAddition(addPoint, objectAddition)


    def executeFeatureAddition(self, featureID, additionArray):
        self.buildPath(featureID)
        model = self.readModel()
        index = self.readIndexLookup()
        props = self.readProps()
        
        if eval("'sub' in model"+self._stringPath):
            exec("model"+self._stringPath+"['sub'].append("+additionArray[0]+")")
            exec("index['mappings'].append("+additionArray[2]+')')
            exec("props['properties'].append("+additionArray[1]+')')
        else:
            exec("model"+self._stringPath+"['sub'] = ["+additionArray[0]+"]")
            exec("index['mappings'].append("+additionArray[2]+')')
            exec("props['properties'].append("+additionArray[1]+')')
        self.saveModel(model)
        self.saveIndexLookup(index)
        self.saveProps(props)
        print(f'**Feature added to ({featureID})**')
        


    def alter_feature(self, alterParamSet):
        self.executeFeatureAlter(alterParamSet)


    def executeFeatureAlter(self, alterParamSet):
        featureID = alterParamSet.pop()
        
        featureExists = self.verifyFeatureExistence(featureID)
        if featureExists:
            props = self.readProps()
            modesAllowed = ['static','dynamic']
            timesAllowed = ['early', 'late']
            for prop in props['properties']:
                if prop['id'] == featureID:
                    for param in alterParamSet:
                        paramSplit = param.split('=')
                        if paramSplit[0] == 'time':
                            if paramSplit[1].lower() in timesAllowed:
                                prop['props'][paramSplit[0]] = paramSplit[1].lower().capitalize()
                            else:
                                print(f'Invalid Value. Binding time value cannot be set to {paramSplit[1].lower()}')
                        elif paramSplit[0] == 'mode':
                            if paramSplit[1].lower() in modesAllowed:
                                prop['props'][paramSplit[0]] = paramSplit[1].lower().capitalize()
                            else:
                                print(f'Invalid Value. Binding mode value cannot be set to {paramSplit[1].lower()}')

            self.saveProps(props)
            print(f'Feature **{featureID}** altered successfully')
        else:
            print(f'Command Execution Failed: Attempting to alter nonexistent feature with ID: {featureID}')


    def resetPath(self):
        self._stringPath = ""
        self._traverseGuide = []
        self._lookupDataSet = []

    def show(self, param):
        if param == "all":
            self.treePrint()
            print("\n\tNotation:[ + = Mandatory, # = Alternative, o = Optional, OR = &, x = Static, > = Dynamic, * = Off ]")
        elif param == "config":
            self.treePrint(param)
            print("\n\tNotation:[ + = Mandatory, # = Alternative, o = Optional, OR = &, x = Static, > = Dynamic, * = Off ]")
        else:
            self.buildPath(param)
            model = self.readModel()
            featureReq = eval("model"+self._stringPath)
            self.printSubtree(featureReq, filter=None)
            print("\n\tNotation:[ + = Mandatory, # = Alternative, o = Optional, OR = &, x = Static, > = Dynamic, * = Off ]")

    def activate_config(self, modelName):
        engState = self.getEngineState()
        for project in engState['projects']:
            if project['name'] == modelName:
                project['status'] = 1
            else:
                project['status'] = 0
        print(f'{modelName} project loaded successfully')
        self.saveEngineState(engState)


    def create_default_config(self, modelName):
        if os.path.exists(self._usrProjectsDir+ "\\"+ modelName):
            print("Model name already exists. Choose another model name")
        else:
            #Initialise new model
            projFolder = self._usrProjectsDir+ "\\"+ modelName
            os.mkdir(projFolder)
            srcFile = os.path.dirname(os.path.abspath('../model/usr/base/model.json'))
            srcFileBase = os.path.join(srcFile, 'model.json')
            srcFileIndex = os.path.join(srcFile, 'index.json')
            srcFileProps = os.path.join(srcFile, 'config.json')
            shutil.copy(srcFileBase, projFolder)
            shutil.copy(srcFileIndex, projFolder)
            shutil.copy(srcFileProps, projFolder)
            #Update Engine State
            self.updateDslState(modelName)
            print(f'{modelName} project created successfully')


    def enforceXORConstraint(self, featureID):
        parentID = self.getFeatureParent(featureID)
        childrenIDArray = self.getFeatureChildren(parentID)
        cleanChildren = []
        for child in childrenIDArray:
            if child != featureID:
                cleanChildren.append(child)
        
        model = self.readModel()
        for cleanChild in cleanChildren:
            self.buildPath(cleanChild)
            exec("model"+self._stringPath+"['props']['status'] = False")

        self.saveModel(model)


    def toggle(self, featureID):
        props = self.readProps()
            
        for prop in props['properties']:
            if prop['id'] == featureID:
                if prop['props']['status'] == True:
                    prop['props']['status'] = False
                    print(f'Feature {featureID} toggled OFF')
                else:
                    prop['props']['status'] = True
                    print(f'Feature {featureID} toggled ON')
        #Save update            
        self.saveProps(props)


    def getFeatureParent(self, featureId):
        lookupData = self.readIndexLookup()
        for featureState in lookupData['mappings']:
            if featureState['child'] == featureId:
                return featureState['parent']


    def getFeatureChildren(self, parentId):
        childrenArray = []
        lookupData = self.readIndexLookup()
        for featureState in lookupData['mappings']:
            if featureState['parent'] == parentId:
                childrenArray.append(featureState['child'])
        return childrenArray


    def getActiveModel(self):
        engstate = self.getEngineState()
        for project in engstate['projects']:
            if project['status'] == 1:
                return project['name']


    def updateDslState(self, pname):
        confid = uuid.uuid1()
        stateStore = {"id": confid.hex, "name": pname, "mode":"", "time":"", "status": 0}
        estate = self.getEngineState()
        estate['projects'].append(stateStore)
        self.saveEngineState(estate)


    def loadLookupData(self):
        self._lookupDataSet = self.readIndexLookup()


    def buildPath(self, entryPoint):
        self.resetPath()
        self.loadLookupData()
        topOfTree = "root_bot"
        currentPtr = entryPoint
        
        while currentPtr != topOfTree:
            newCurrentPtr = self.findNextStep(currentPtr)
            currentPtr = newCurrentPtr
        
        self._traverseGuide.reverse()
        if len(self._traverseGuide) > 0:
            self.buildStringPath()
        else:
            print("Traverse path empty")
            
    
    def findNextStep(self, currentStep):
        for row in self._lookupDataSet['mappings']:
            if row['child'] == currentStep:
                self._traverseGuide.append(currentStep)
                newPointer = row['parent']
                return newPointer


    def buildStringPath(self):
        featureLevel = self.readModel()
        for featureID in self._traverseGuide:
            newFeatureLevel = self.findFeaturePosition(featureLevel, featureID)
            featureLevel = newFeatureLevel
        return self._stringPath


    def findFeaturePosition(self, featureLevel, featureID):
        for idx, item in enumerate(featureLevel['sub']):
            if featureLevel['sub'][idx]['id'] == featureID:
                self._stringPath = self._stringPath + "['sub']["+str(idx)+"]"
                return featureLevel['sub'][idx]

    
    def man(self, queryCommand):
        if queryCommand == 'all':
            self.fullManual()
        else:
            self.subsetManual(queryCommand)

    def getProperties(self, featureID):
        allProps = self.readProps()

        for prop in allProps['properties']:
            if prop['id'] == featureID:
                return prop


    def treePrint(self, filter=None):
        activeModel = self.readModel()
        if "sub" in activeModel:
            self.printSubtree(activeModel, filter)

    def printSubtree(self, subArray, filter, level='', desc='', group='', mode='', stat=''):
        propGet = self.getProperties(subArray['id'])
        
        if propGet != None:
            if subArray['rel'] == "MAN":
                    desc += "+"
            elif subArray['rel'] == "OPT":
                    desc += "o"

            if subArray['group'] == "XOR":
                group += "x"
            elif subArray['group'] == "OR":
                group += '#'
            elif subArray['group'] == "AND":
                group += "&"

            if propGet['props']['mode'] == "Static":
                    mode += ">"
            elif propGet['props']['mode'] == "Dynamic":
                    mode += ">>"

            if propGet['props']['status'] == False:
                stat = '*'

        if filter == "config" and propGet != None:
            if propGet['props']['status'] == True:   
                print(f'{level}|{desc}--{mode}--{group} '+ subArray['name'] + '--{id: '+ subArray['id']+'}'+f'{stat}')

        if filter == None:
            print(f'{level}|{desc}--{mode}--{group} '+ subArray['name'] + '--{id: '+ subArray['id']+'}'+f'{stat}') 
        
        if "sub" in subArray:
            level += '\t'
            for sub in subArray['sub']:
                self.printSubtree(sub, filter, level)

    def config(self, commandArray):
        print(commandArray[0])
        return commandArray[0]+'_'+commandArray[1]

    def config_mode_set(self, commandArray):
        cmdParam = commandArray[1]
        if cmdParam.lower() == "early" or cmdParam.lower() == "late":
            engstate = self.getEngineState()
            for project in engstate['projects']:
                if project['status'] == 1:
                    project['mode'] = cmdParam.lower()
            self.saveEngineState(engstate)
        else:
            print("Invalid mode parameter. Parameter must be set to either 'early' or 'late'")

    def select(self, cmdList):
        selectPoints = []
        selStatement = cmdList.pop(0)
        allprops = self.readProps()
        pindex = 0

        for prop in allprops['properties']:
            if prop['id'] in cmdList:
                selectPoints.append(pindex)
            pindex += 1

        updatedProps = self.bulkSelect(allprops, selectPoints)
        self.saveProps(updatedProps)

    def bulkSelect(self, allprops, popIndexList):
        for index in popIndexList:
            allprops['properties'][index]['props']['status'] = True
            featureId = allprops['properties'][index]['id']
            print(f'---Feature {featureId} selected')
        return allprops

    def load(self, featureID):
        props = self.readProps()
        #get topic from registry
        topicEndpoint = self.readRegistry(featureID)
        if topicEndpoint != None:
            #broadcast on topic
            r = rospy.Rate(1)
            pub = rospy.Publisher(topicEndpoint, String, queue_size=1)
            #check node life
            nodelist = rosnode.get_node_names()
            
            thisNode = "/"+topicEndpoint
            #check if exists in rosnode list
            if thisNode not in nodelist:
                #check binding and update list
                canBeLoaded = self.checkBindingCombination(featureID, action="load")
                #if feature can be possibly loaded
                if canBeLoaded == True:
                    pub.publish("load")
                    #change config status    
                    for prop in props['properties']:
                        if prop['id'] == featureID:  
                            prop['props']['status'] = False
                            print(f'Feature {featureID} has been loaded successfully')   
                    #Save update to model           
                    self.saveProps(props)
                    #update server param of bound features
                    current_server_state = list(rospy.get_param('/global_config_param'))
                    
                    if featureID not in current_server_state:
                        self.reactivateFeature(topicEndpoint)
                        current_server_state.append(featureID)
                        rospy.set_param('/global_config_param', current_server_state)
                        
                    else:
                        print("Feature already exists in server config. No need to re-set it")
                else:
                    bt = rospy.get_param('/binding_time')
                    print(f'Feature {featureID} cannot be loaded at {bt} time')
            else:
                print(f'Feature node {featureID} already loaded')

            r.sleep()
        else:
            print("Node cannot be found in registry")


    def unload(self, featureID):
        props = self.readProps()
        #get topic from registry
        topicEndpoint = self.readRegistry(featureID)
        if topicEndpoint != None:
            #broadcast on topic
            r = rospy.Rate(1)
            pub = rospy.Publisher(topicEndpoint, String, queue_size=1)
            #check node life
            nodelist = rosnode.get_node_names()
            
            thisNode = "/"+topicEndpoint
            #check if exists in rosnode list
            if thisNode in nodelist:
                #check binding and update list
                canBeUnloaded = self.checkBindingCombination(featureID, action="unload")
                #if feature can be possibly loaded
                if canBeUnloaded == True:
                    pub.publish("unload")
                    #change config status    
                    for prop in props['properties']:
                        if prop['id'] == featureID:  
                            prop['props']['status'] = False
                            print(f'Feature {featureID} has been unloaded successfully')   
                    #Save update to model           
                    self.saveProps(props)
                    #update server param of bound features
                    current_server_state = list(rospy.get_param('/global_config_param'))
                    
                    if featureID in current_server_state:
                        current_server_state.remove(featureID)
                        rospy.set_param('/global_config_param', current_server_state)
                else:
                    bt = rospy.get_param('/binding_time')
                    print(f'Feature {featureID} cannot be unloaded at {bt} time')
            else:
                print(f'Feature node {featureID} already unloaded')

            r.sleep()
        else:
            print("Node cannot be found in registry")
        
    def ping(self, featureID):
        #get topic from registry
        topicEndpoint = self.readRegistry(featureID)
        #broadcast on topic
        r = rospy.Rate(1)
        pub = rospy.Publisher(topicEndpoint, String, queue_size=1)
        #check node life
        nodelist = rosnode.get_node_names()
        thisNode = "/"+topicEndpoint
        
        #check if exists in rosnode list
        if thisNode in nodelist:
            pub.publish("ping")
        else:
            print(f'Ping failed. Feature {featureID} unloaded')

        r.sleep()    

    def validate_config(self):
        activeM = self.getActiveModel()
        print(f'+++ Validating Configuration: {activeM}.....+++\n')
        
        props = self.readProps()
        #pass configuration to configurator
        self.checkAllConstraints(props)
        self.printViolations()

        print(f'\n+++ Configuration ({activeM}) Validation Complete +++')

    def run_config(self):
        currentConfig = self.dumpCurrentConfig()
       
        #set global param list
        rospy.set_param('global_config_param', currentConfig)
        rospy.set_param('binding_time', "late")

        if rospy.get_param('/global_config_param'):
            rospy.loginfo('Early binding executed successfully')
        
        self.node_template_spawn(currentConfig)

    def node_template_spawn(self, nodeList):
        reg = self.getRegistryState()
        
        for vars in reg['env_var']:
            if vars['fid'] not in nodeList:
                self.unload(vars['fid'])  


    def dump_server_settings(self):
        reg = self.getEngineState()
        for project in reg['projects']:
            if project['status'] == 1:
                configName = project['name']

        binding_time = rospy.get_param('/binding_time')
        print("------------------------------------------------")
        print(f'Config. Name: {configName}   |    Binding time: {binding_time}')
        print("------------------------------------------------")

        if rospy.get_param('/global_config_param'):
            for feature in rospy.get_param('/global_config_param'):
                print(feature)


    def reactivateFeature(self, nodeName):
        package = 'ros_confapp'
        executable = nodeName
        node = roslaunch.core.Node(package=package, node_type=executable, name=executable, machine_name=None, output='screen')

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        rospy.loginfo(f'Node {nodeName} Started Successfully')
        launch.launch(node)

    def set_include(self, setList):
        propsList = self.readProps()

        for prop in propsList['properties']:
            if prop['id'] == setList[0]:
                if setList[1] not in prop['constraints']['inc']:
                    if setList[1] not in prop['constraints']['ex']:
                        prop['constraints']['inc'].append(setList[1])
                        self.saveProps(propsList)
                        print(f'{setList[1]} added as include constraint to feature {setList[0]} successfully')
                    else:
                        print(f'Include you are attempting to set, already exists as exclude')

    
    def set_exclude(self, setList):
        propsList = self.readProps()

        for prop in propsList['properties']:
            if prop['id'] == setList[0]:
                if setList[1] not in prop['constraints']['ex']:
                    if setList[1] not in prop['constraints']['inc']:
                        prop['constraints']['ex'].append(setList[1])
                        self.saveProps(propsList)
                        print(f'{setList[1]} added as exclude constraint to feature {setList[0]} successfully')
                    else:
                        print(f'Exclude you are attempting to set, already exists as include')
        
        

    



