#!/usr/bin/env python3

from ros_confapp.dsl.bindings import Bindings


class Config(Bindings):
    def __init__(self):
        Bindings.__init__(self)
        self.boundFeaturesQueue = Bindings.getAllBindings(self)


class Core(Config):
    def __init__(self):
        Config.__init__(self)

    def validateTimeModeCombo(self, cmd):
        dslComms = cmd.split()
        
        if dslComms[1].strip() in self.boundFeaturesQueue:
            if dslComms[0] == "load" or dslComms[0] == "unload":
                #check feature props against binding combos
                permission = self.checkBindingCombination(dslComms[1].strip().lower(), dslComms[0].strip().lower())
                if permission:
                    return True
                else:
                    return False
        else:
            print("Sorry, abstract feature binding cannot be altered")

