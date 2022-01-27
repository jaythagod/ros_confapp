#!/usr/bin/env python
from ros_confapp.dsl.constraintChecker import ConstraintChecker
from ros_confapp.dsl.bindings import Bindings


class Configurator(ConstraintChecker, Bindings):
    def __init__(self):
        ConstraintChecker.__init__(self)
        Bindings.__init__(self)

    def checkAllConstraints(self, configuration):
        # clear previous validation data
        self.resetViolationsList()
        #loop through config
        for config in configuration:
            #for each config entry, check includes
            if len(config['constraints']['inc']) > 0:
                self.includes(config['id'], config['constraints']['inc'])
            #for each config entry, check excludes
            if len(config['constraints']['ex']) > 0:
                self.excludes(config['id'], config['constraints']['ex'])
            
            self.featureConfigurationConstraints(config['id'], config['props']['mode'], config['props']['time'], config['constraints']['tbind'], config['constraints']['mbind'])
        #check parent child constraint
        self.parentChildMode()

    def printViolations(self):
        if(len(self.includeViolations) > 0):
            inc_string = "\n".join(self.includeViolations)
            print(f'\n\tInclude Constraint Violation:\n ----------------------------------------------------- \n        Feature      |       Inclusion\n -----------------------------------------------------')
            print(inc_string)
            print("------------------------------------------------------")

        if(len(self.excludeViolations) > 0):
            ex_string = "\n".join(self.excludeViolations)
            print(f'\n\tExclude Constraint Violation:\n ----------------------------------------------------- \n        Feature      |       Exclusion\n -----------------------------------------------------')
            print(ex_string)
            print("------------------------------------------------------")
        
        if(len(self.parentChildViolations) > 0):
            pairString_list = [pair[0]+" | "+pair[1]+"\n--------------------------------------------" for pair in self.parentChildViolations]
            pc_list = "\n".join(pairString_list)
            print(f'\n\tParent/Child Constraint Violation:\n ------------------------------------------- \n         Parent      |       Child\n -------------------------------------------')
            print(pc_list)

        if(len(self.bindingConstraintViolations[0]) > 0):
            time_string = "\n".join(self.bindingConstraintViolations[0])
            print(f'\n\tTime Binding Mismatch Violation:\n ----------------------------------------------------- \n')
            print(time_string)
            print("------------------------------------------------------")

        if(len(self.bindingConstraintViolations[1]) > 0):
            mode_string = "\n".join(self.bindingConstraintViolations[1])
            print(f'\n\tMode Binding Mismatch Violation:\n ----------------------------------------------------- \n')
            print(mode_string)
            print("------------------------------------------------------")
