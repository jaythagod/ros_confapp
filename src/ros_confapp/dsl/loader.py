#!/usr/bin/env python
import rospy
import rostopic
from ros_confapp.srv import processConfigAction, processConfigActionRequest, processConfigActionResponse
import sys

try:
    import readline
except ImportError:
    import pyreadline as readline

from ros_confapp.dsl.engine import Engine

class PromptSelecter(Engine):
    def __init__(self):
        Engine.__init__(self)
        self._consolePrompt = "\ndcflib"
        self.appMode = ""
        

    def buildPromptActiveConfig(self):
        self.appModeCheck()
        self._consolePrompt += "@"+self.appMode.upper()
        engState = self.getEngineState()
        for project in engState['projects']:
            if project['status'] == 1:
                currentActiveConfig = "["+project['name']+"]"
                self._consolePrompt += currentActiveConfig
        self._consolePrompt += ">>"

    def appModeCheck(self):
        try:
            rostopic.get_topic_class('/rosout')
            self.appMode = "late"
        except rostopic.ROSTopicIOException as masterNotFound:
            self.appMode = "early"




class Loader(Engine):
    def __init__(self):
        Engine.__init__(self)

    def cmdServiceStringPrep(self, cmdList):
        return cmdList[0]+" "+cmdList[1]
        

    def sanitizeCommand(self, cmd):
        sanitized = cmd.strip().lower()
        return sanitized.split()

    def sendRequestOverService(self, commandExecMessage):

        rospy.init_node("dsl_command_exec")
        rospy.wait_for_service('get_cmd')
        
        try:
            
            core_config = rospy.ServiceProxy('get_cmd', processConfigAction)
            respl = core_config(commandExecMessage)
            return respl.feedback
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed")
            rospy.loginfo(e)


    def getListOfFeatureIds(self):
        allProps = self.readProps()
        allIds = [prop['id'] for prop in allProps['properties']]
        return allIds


    def idCompleter(self, text, state):
        fids = self.getListOfFeatureIds()
        options = [fid for fid in fids if fid.startswith(text)]
        if state < len(options):
            return options[state]
        else:
            return None

    
    def launch(self):
        #TODO:print opening message, credit and short guide
        readline.parse_and_bind("tab: complete")
        readline.set_completer(self.idCompleter)

        if len(sys.argv) < 2:
        #activate console mode
            while(True):
                lateCommands = ['load','unload']
                prompt = PromptSelecter()
                prompt.buildPromptActiveConfig()
                cmdline = input(prompt._consolePrompt)
                
                if not cmdline or cmdline[0] == '#' or len(cmdline) == 0:
                        continue
                    #handle exit command
                elif cmdline.strip().lower() == 'exit':
                    confirmation = self.sanitizeCommand(input(prompt._consolePrompt +" Confirm exit (y/n): "))
                    if confirmation[0] == 'y' or confirmation[0] =='Y':
                        break
                    elif confirmation[0] != 'n' and confirmation[0] != 'y':
                        print("Invalid exit confirmation value provided. Try again")
                else:
                    cmd = self.sanitizeCommand(cmdline)
                    if cmd[0] not in lateCommands:
                        self.interpret(cmd)
                    prepedCmdString = self.cmdServiceStringPrep(cmd)
                    self.sendRequestOverService(prepedCmdString)
            
            else:
                pass
                #handle invalid commands
                #print(f'Invalid Command {cmd[0]}')
        
        elif len(sys.argv) == 2:
             with open(sys.argv[1], 'r') as dslfile:
                for cmdline in dslfile:
                    
                    #ignore comment line and blank lines
                    if not cmdline or cmdline[0] == '#' or len(cmdline.strip()) == 0:
                        continue
                    else:
                        cmd = self.sanitizeCommand(cmdline)
                        self.interpret(cmd)
                        return cmdline


        elif len(sys.argv) > 2:
            print('UsageError: unhandled usage mode. Refer to manual for more details')
            sys.exit(1)