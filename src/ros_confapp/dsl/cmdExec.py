#!/usr/bin/env python
import os
import shutil
import uuid
from distutils.util import strtobool

from ros_confapp.dsl.documentation import Documentation
from ros_confapp.dsl.dslState import DslState


#from dsl.config import Configuration

class CmdExec(DslState, Documentation):
    def __init__(self):
        DslState.__init__(self)
        Documentation.__init__(self)
        self._usrProjectsDir = os.path.dirname(os.path.abspath('../model/usr/base'))
        self._usrLaunchDir = os.path.dirname(os.path.abspath('../launch/base.launch'))
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
        #check if 'sub' key exists in model object
        #TODO: Enforce XOR in alternative features
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
            
            for prop in props['properties']:
                if prop['id'] == featureID:
                    for param in alterParamSet:
                        paramSplit = param.split('=')
                        if paramSplit[0] == 'type':
                            prop['props'][paramSplit[0]] = paramSplit[1].lower().capitalize()
                        elif paramSplit[0] == 'rel':
                            prop['props']['relationship'] = paramSplit[1].upper()
                            if paramSplit[1].upper() == 'OR':
                                prop['props']['description'] = 'Optional'
                            elif paramSplit[1].upper() == 'AND':
                                prop['props']['description'] = 'Mandatory'
                            elif paramSplit[1].upper() == 'XOR':
                                prop['props']['description'] = 'Alternative'
                        elif paramSplit[0] == 'time':
                            prop['props'][paramSplit[0]] = paramSplit[1].lower().capitalize()
                        elif paramSplit[0] == 'mode':
                            prop['props'][paramSplit[0]] = paramSplit[1].lower().capitalize()
                        elif paramSplit[0] == 'status':
                            statusVal = strtobool(paramSplit[1].lower())
                            prop['props'][paramSplit[0]] = bool(statusVal)


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
            print("\n\tNotation:[ + = Mandatory, # = Alternative, o = Optional, x = Static, > = Dynamic, * = Off ]")
        else:
            self.buildPath(param)
            model = self.readModel()
            featureReq = eval("model"+self._stringPath)
            self.printSubtree(featureReq)
            print("\n\tNotation:[ + = Mandatory, # = Alternative, o = Optional, x = Static, > = Dynamic, * = Off ]")

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
            launchFile = self._usrLaunchDir+"\\"+modelName+".launch"
            os.mkdir(projFolder)
            srcFile = os.path.dirname(os.path.abspath('../model/usr/base/base.json'))
            srcFileBase = os.path.join(srcFile, 'base.json')
            srcFileIndex = os.path.join(srcFile, 'index.json')
            srcFileProps = os.path.join(srcFile, 'props.json')
            shutil.copy(srcFileBase, projFolder)
            shutil.copy(srcFileIndex, projFolder)
            shutil.copy(srcFileProps, projFolder)
            #Create extracts folder
            snippetsSrc = os.path.dirname(os.path.abspath('../featx/extracts/base/main.py'))
            dstMain = os.path.dirname(os.path.abspath('../featx/extracts/base'))
            snippetsDst = dstMain+ "\\"+ modelName
            shutil.copytree(snippetsSrc, snippetsDst)
            #create project launch file
            try:
                open(launchFile, 'a').close()
            except OSError:
                print('Launch file creation failed')

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
        stateStore = {"id": confid.hex, "name": pname, "mode":"", "status": 0}
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


    def treePrint(self):
        activeModel = self.readModel()
        if "sub" in activeModel:
            self.printSubtree(activeModel)

    def printSubtree(self, subArray, level='', desc='', mode='', stat=''):
        propGet = self.getProperties(subArray['id'])
        
        if propGet != None:
            if propGet['props']['relationship'] == "AND":
                    desc += "+"
            elif propGet['props']['relationship'] == "OR":
                    desc += "o"
            elif propGet['props']['relationship'] == "XOR":
                    desc += "#"

            if propGet['props']['mode'] == "Static":
                    mode += "x"
            elif propGet['props']['mode'] == "Dynamic":
                    mode += ">"

            if propGet['props']['status'] == False:
                stat = '*'
           
        print(f'{level}|{desc}--{mode} '+ subArray['name'] + '--{id: '+ subArray['id']+'}'+f'{stat}')
        
        if "sub" in subArray:
            level += '\t'
            for sub in subArray['sub']:
                self.printSubtree(sub, level)

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
            
        for prop in props['properties']:
            if prop['id'] == featureID:  
                prop['props']['status'] = True
                print(f'Feature {featureID} loaded')   
        #Save update            
        self.saveProps(props)


    def unload(self, featureID):
        props = self.readProps()
            
        for prop in props['properties']:
            if prop['id'] == featureID:  
                prop['props']['status'] = False
                print(f'Feature {featureID} unloaded')   
        #Save update            
        self.saveProps(props)

