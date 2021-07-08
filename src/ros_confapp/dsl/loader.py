#!/usr/bin/env python
import sys

from ros_confapp.dsl.engine import Engine


class Loader(Engine):
    def __init__(self):
        Engine.__init__(self)
        self._consolePrompt = "dcflib >> "

    def sanitizeCommand(self, cmd):
        sanitized = cmd.strip().lower()
        return sanitized.split()

    def launch(self):
        if len(sys.argv) < 2:
        #activate console mode
            while(True):
                dcflibPrompt = "dcflib >> "
                cmd = self.sanitizeCommand(input(self._consolePrompt)) 
                if cmd[0] in self.dslCommands:
                    #handle exit command
                    if cmd[0] == 'exit':
                        confirmation = self.sanitizeInputs(input(self._consolePrompt +" Confirm exit (y/n): "))
                        if confirmation == 'y':
                            break
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