#!/usr/bin/env python
import os
import shutil

from ros_confapp.dsl.dslState import DslState

#from ros_confapp.dsl.documentation import Documentation
#from dsl.config import Configuration

class CmdExec(DslState):
    def __init__(self):
        DslState.__init__(self)
        #Documentation.__init__(self)
        self._usrProjectsDir = os.path.dirname(os.path.abspath('../model/usr/base'))
        self._stringPath = ""
        self._traverseGuide = []
        self._lookupDataSet = []

    def ls(self):
        engState = self.getEngineState()
        for proj in engState['projects']:
            if proj['status'] == 1:
                print('|')
                print('|--'+ proj['name']+' [status=Active][mode='+proj['mode']+']')
            else:
                print('|')
                print('|--'+ proj['name'])

    def remove(self, paramList):
        model = self.readModel()
        remCmdList = self.validateRemoveCommandStructure(paramList)
        if len(remCmdList[0]) > 0:
            exec("del model"+remCmdList[1])
        self.saveModel(model)
        print(f'--Feature({remCmdList[0][1]}) removed')

    def validateRemoveCommandStructure(self, dslCommand):
        removeRule = self.returnSingleLexRule("remove")
        if len(dslCommand) == removeRule['length']:
            self.buildPath(dslCommand[1])
            return [dslCommand, self._stringPath]
        else:
            self.printError(50)

    def add(self, addPoint, objectAddition):
        self.executeFeatureAddition(addPoint, objectAddition)

    def executeFeatureAddition(self, featureID, additionArray):
        self.buildPath(featureID)
        model = self.readModel()
        index = self.readIndexLookup()
        #check if 'sub' key exists in model object
        #TODO: Enforce XOR in alternative features
        if eval("'sub' in model"+self._stringPath):
            exec("model"+self._stringPath+"['sub'].append("+additionArray[0]+")")
            exec("index['mappings'].append("+additionArray[1]+')')
        else:
            exec("model"+self._stringPath+"['sub'] = ["+additionArray[0]+"]")
            exec("index['mappings'].append("+additionArray[1]+')')
        self.saveModel(model)
        self.saveIndexLookup(index)
        print(f'**Feature added to ({featureID})**')

    def show(self, param):
        if param == "all":
            self.treePrint()
            print("\n\tNotation:[ + = Mandatory, # = Alternative, o = Optional, x = Static, > = Dynamic ]")
        else:
            self.buildPath(param)
            model = self.readModel()
            featureReq = eval("model"+self._stringPath)
            self.printSubtree(featureReq)
            print("\n\tNotation:[ + = Mandatory, # = Alternative, o = Optional, x = Static, > = Dynamic ]")

    def load(self, modelName):
        engState = self.getEngineState()
        for project in engState['projects']:
            if project['name'] == modelName:
                project['status'] = 1
                print(f'{modelName} project loaded successfully')
            project['status'] = 0
        self.saveEngineState(engState)

    def create(self, modelName):
        if os.path.exists(self._usrProjectsDir+ "\\"+ modelName):
            print("Model name already exists. Choose another model name")
        else:
            projFolder = self._usrProjectsDir+ "\\"+ modelName
            os.mkdir(projFolder)
            srcFile = os.path.dirname(os.path.abspath('../model/usr/base/base.json'))
            srcFileBase = os.path.join(srcFile, 'base.json')
            srcFileIndex = os.path.join(srcFile, 'index.json')
            shutil.copy(srcFileBase, projFolder)
            shutil.copy(srcFileIndex, projFolder)
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

        self.buildPath(featureID)
        model = self.readModel()
       
        featStatus = eval("model"+self._stringPath+"['props']['status']")

        if featStatus == True:
            #toggle feature status off
            exec("model"+self._stringPath+"['props']['status'] = False")
            action = 'off'
        else:
            #toggle feature status on
            exec("model"+self._stringPath+"['props']['status'] = True")
            action = 'on'
        self.saveModel(model)
        print(f'**Feature({featureID}) toggled {action}**')

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
        stateStore = {"name": pname, "mode":"", "status": 0}
        estate = self.getEngineState()
        estate['projects'].append(stateStore)
        self.saveEngineState(estate)


    def loadLookupData(self):
        self._lookupDataSet = self.readIndexLookup()

    def buildPath(self, entryPoint):
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

    def treePrint(self):
        activeModel = self.readModel()
        if "sub" in activeModel:
            self.printSubtree(activeModel)

    def printSubtree(self, subArray, level='', desc='', mode=''):
        if "props" in subArray:
            if subArray['props']['relationship'] == "AND":
                    desc += "+"
            elif subArray['props']['relationship'] == "OR":
                    desc += "o"
            elif subArray['props']['relationship'] == "XOR":
                    desc += "#"

            if subArray['props']['mode'] == "Static":
                    mode += "x"
            elif subArray['props']['mode'] == "Dynamic":
                    mode += ">"
        
        print(f'{level}|{desc}--{mode} '+ subArray['name'] + '--{id: '+ subArray['id']+'}')
        
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

    def unpackAndSaveIndexLookup():
        pass
