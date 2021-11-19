#!/usr/bin/env python
import random
from ros_confapp.dsl.dslState import DslState
from ros_confapp.dsl.cmdExec import CmdExec
from ros_confapp.dsl.errorCodes import ErrorCodes

cmdexec = CmdExec()

class Engine(DslState, ErrorCodes):
    def __init__(self):
        DslState.__init__(self)
        ErrorCodes.__init__(self)

    def interpret(self, cmd):
        if self.lexicalStructure(cmd):
            #create and load commands doesnot require an active project selection
            self.grammar(cmd)
            if cmd[0] == "create_default_config" or cmd[0] == "activate_config":
                self.runCommand(cmd)
            else:
                if self.checkActiveProject():
                    intResult = self.runCommand(cmd)
                    return intResult
                else:
                    self.printError(1000)
        else:
            self.printError(10)


    def lexicalStructure(self, cmd):
        rules = self.getLexicalRules()
        for lexicon in rules['lexicons']:
            if cmd[0] == lexicon["keyword"]:
                return True
        return False

    def checkActiveProject(self):
        estate = self.getEngineState()
        for project in estate["projects"]:
            if project["status"] == 1:
                return True
        return False

    def getActiveProject(self):
        estate = self.getEngineState()
        for project in estate["projects"]:
            if project["status"] == 1:
                return project
        return None

    def grammar(self, cmd):
        eobj = self.getCommandDetails(cmd)
        if eobj != False:
            #check length
            if len(cmd) != eobj['length']:
                return False
            #check params
            if len(eobj['params']) > 0:
                #check required params against passed ones
                pass
            #check
            pass

    def validateAddCommandStructure(self, dslCommand):
        if dslCommand[0] == 'add_feature':
            if 'to' not in dslCommand:
                self.printError(20)
            else:
                #return array of feature, props and index object
                featurePropsIndexObject = self.buildAddFeatureObject(dslCommand[-1], dslCommand[1])
                return featurePropsIndexObject
    
    def validateAlterFeature(self, dslCommand):
        paramsCheck = []
        allowedTypes = ['concrete','abstract']
        allowedRels = ['man','opt']
        allowedGroups = ['or','xor']
        allowedModes = ['static','dynamic']
        allowedTimes = ['early','late']
        allowedStatus = ['true', 'false']
        alterRule = self.returnSingleLexRule("alter_feature")
        #check for invalid command parameters
        for element in dslCommand:
            result = element.split('=')
            if len(result) == 2:
                if result[0] in alterRule['params']:
                    #check command parameter values
                    if result[0] == "type":
                        if result[1] in allowedTypes:
                            paramsCheck.append(element)
                        else:
                            print(f'List of allowed type values: {allowedTypes}')

                    if result[0] == "rel":
                        if result[1] in allowedRels:
                            paramsCheck.append(element)
                        else:
                            print(f'List of allowed rel values: {allowedRels}')

                    if result[0] == "group":
                        if result[1] in allowedGroups:
                            paramsCheck.append(element)
                        else:
                            print(f'List of allowed group values: {allowedGroups}')

                    if result[0] == "mode":
                        if result[1] in allowedModes:
                            paramsCheck.append(element)
                        else:
                            print(f'{element} not a valid mode value. Allowed mode values are: {allowedModes}')

                    if result[0] == "time":
                        if result[1] in allowedTimes:
                            paramsCheck.append(element)
                        else:
                            print(f'{element} not a valid mode value. Allowed mode values are: {allowedTimes}')

                    if result[0] == "status":
                        if result[1] in allowedStatus:
                            paramsCheck.append(element)
                        else:
                            print(f'List of allowed status values: {allowedStatus}')
                else:
                    print(f'{result[0]} is not a valid alter_feature command parameter')
                    return "val_error"
        paramsCheck.append(dslCommand[1])
        return paramsCheck

    def validateIncludeExclude(self, dslCommandString):
        #grab command string from list
        commandArray = dslCommandString[1::]
       
        #check id existence
        for feature in commandArray:
            testResults = self.verifyFeatureExistence(feature)
            if testResults == False:
                print(f'Feature {feature} , is not a valid feature ID')
                return "invalid_comm"
        return commandArray


    def buildAddFeatureObject(self, parentID, childName):
        
        generatedChildFeatureID = self.generateNewUniqueFeatureID(parentID)

        featureStringBuild = f'{{"id":"{generatedChildFeatureID}", "name": "{childName}", "type": "Concrete", "group": "AND", "rel":"MAN"}}'
        propsStringBuild = f'{{"id":"{generatedChildFeatureID}","constraints": {{"inc": [], "ex": []}}, "props": {{"mode":"Static", "time":"Early", "status": False}} }}'
        indexStringBuild = f'{{"parent":"{parentID}", "child":"{generatedChildFeatureID}"}}'
        return [featureStringBuild, propsStringBuild, indexStringBuild]      

    def generateNewUniqueFeatureID(self, parent):
        parent = parent +'abcdefghijklmnopqrstuvxyz'
        parentSplit = parent.split('_')
        pname = parentSplit[0]
        newIdStrBase = list(parentSplit[1])
        random.shuffle(newIdStrBase)
        newStr = ''.join(newIdStrBase)
        newStr = newStr[::5]
        return newStr+'_'+pname

    def runCommand(self, cmd):
        if cmd[0] == "create_default_config":
            getattr(cmdexec, cmd[0])(cmd[1])

        if cmd[0] == "activate_config":
            getattr(cmdexec, cmd[0])(cmd[1])

        if cmd[0] == "show":
            getattr(cmdexec, cmd[0])(cmd[1])

        if cmd[0] == "man":
            getattr(cmdexec, cmd[0])(cmd[1])

        if cmd[0] == "add_feature":
            featureBuild = self.validateAddCommandStructure(cmd)
            
            if 'id' in featureBuild[0]:
                getattr(cmdexec, cmd[0])(cmd[-1], featureBuild)
            else:
                self.printError(40)

        if cmd[0] == "alter_feature":
            alterSet = self.validateAlterFeature(cmd)
            getattr(cmdexec, cmd[0])(alterSet)

        if cmd[0] == "ls":
            try:
                getattr(cmdexec, cmd[0])()
                return 1
            except:
                return 0

        if cmd[0] == "toggle":
            isVerified = self.verifyFeatureExistence(cmd[1])
            if isVerified:
                getattr(cmdexec, cmd[0])(cmd[1])
            else:
                print(f'Feature {cmd[1]} does not exist. Enter a valid feature ID')

        if cmd[0] == "remove":
            getattr(cmdexec, cmd[0])(cmd)

        if cmd[0] == "select":
            getattr(cmdexec, cmd[0])(cmd)

        if cmd[0] == "config":
            getattr(cmdexec, cmd[0])(cmd)

        if cmd[0] == "config_mode_set":
            getattr(cmdexec, cmd[0])(cmd)

        if cmd[0] == "load":
            getattr(cmdexec, cmd[0])(cmd[1])

        if cmd[0] == "unload":
            getattr(cmdexec, cmd[0])(cmd[1])

        if cmd[0] == "validate_config":
            getattr(cmdexec, cmd[0])()

        if cmd[0] == "ping":
            getattr(cmdexec, cmd[0])(cmd[1])

        if cmd[0] == "run_config":
            getattr(cmdexec, cmd[0])()

        if cmd[0] == "dump_server_settings":
            getattr(cmdexec, cmd[0])()

        if cmd[0] == "set_include":
            incexSet = self.validateIncludeExclude(cmd)
            getattr(cmdexec, cmd[0])(incexSet)

        if cmd[0] == "set_exclude":
            incexSet = self.validateIncludeExclude(cmd)
            if incexSet != "invalid_comm":
                getattr(cmdexec, cmd[0])(incexSet)