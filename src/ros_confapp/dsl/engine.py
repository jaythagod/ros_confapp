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
            if cmd[0] == "create" or cmd[0] == "load":
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
        #add command block begin
        if dslCommand[0] == 'add':
            if 'to' not in dslCommand:
                self.printError(20)
            else:
                paramsCheck = []
                for element in dslCommand:
                    result = element.split('=')
                    if len(result) == 2:
                        paramsCheck.append(result[0])
                        if result[0] == 'name':
                            name = result[1].capitalize()
                        elif result[0] == 'type':
                            type = result[1].capitalize()
                        elif result[0] == 'rel':
                            rel = result[1].upper()

                addRule = self.returnSingleLexRule("add")
                if paramsCheck == addRule['params']:
                    builtFeatureObjectList = self.buildAddObject(dslCommand[-1], name, type, rel)
                    return builtFeatureObjectList
                else:
                    self.printError(30)


    def buildAddObject(self, parent, name='', type='', rel=''):
        desc = ''
        if rel == 'AND':
            desc = 'Mandatory'
        elif rel == 'OR':
            desc = 'Optional'
        elif rel == 'XOR':
            desc == 'Optional'

        fid = self.generateNewUniqueFeatureID(parent)

        featureStringBuild = f'{{"id":"{fid}", "name": "{name}", "props": {{"type": "{type}", "description": "{desc}", "relationship":"{rel}", "status": True}} }}'
        indexStringBuild = f'{{"parent":"{parent}", "child":"{fid}"}}'
        return [featureStringBuild, indexStringBuild]       

    def generateNewUniqueFeatureID(self, parent):
        parent = parent +'abcdefghijklmnopqrstuv'
        parentSplit = parent.split('_')
        pname = parentSplit[0]
        newIdStrBase = list(parentSplit[1])
        random.shuffle(newIdStrBase)
        newStr = ''.join(newIdStrBase)
        newStr = newStr[::5]
        return newStr+'_'+pname

    def runCommand(self, cmd):
        if cmd[0] == "create":
            getattr(cmdexec, cmd[0])(cmd[2])

        if cmd[0] == "load":
            getattr(cmdexec, cmd[0])(cmd[1])

        if cmd[0] == "show":
            getattr(cmdexec, cmd[0])(cmd[1])

        if cmd[0] == "man":
            getattr(cmdexec, cmd[0])(cmd[1])

        if cmd[0] == "add":
            featureBuild = self.validateAddCommandStructure(cmd)
            if 'id' in featureBuild[0]:
                getattr(cmdexec, cmd[0])(cmd[-1], featureBuild)
            else:
                self.printError(40)

        if cmd[0] == "ls":
            try:
                getattr(cmdexec, cmd[0])()
                return 1
            except:
                return 0

        if cmd[0] == "remove":
            getattr(cmdexec, cmd[0])(cmd)

        if cmd[0] == "config":
            getattr(cmdexec, cmd[0])(cmd)

        if cmd[0] == "config_mode_set":
            getattr(cmdexec, cmd[0])(cmd)
