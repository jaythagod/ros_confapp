from ros_confapp.dsl.dslState import DslState

class CrossFunctionalConstraints(DslState):
    def __init__(self):
        DslState.__init__(self)
        self.includeViolations = []
        self.excludeViolations = []
        self.parentChildViolations = []

    def findFeatureStatus(self, featureID):
        props = self.readProps()
        #pass configuration to configurator
        for prop in props['properties']:
            if prop['id'] == featureID:
                if prop['props']['status'] == True:
                    return "selected"
                else:
                    return "not-selected"

    def resetViolationsList(self):
        self.includeViolations = []
        self.excludeViolations = []
        self.parentChildViolations = []


    def includes(self, feature, includeList):
        mainIncFeatureStatus = self.findFeatureStatus(feature)
        if mainIncFeatureStatus == "selected":
            for inc in includeList:
                statusCheck = self.findFeatureStatus(inc)
                if statusCheck == "not-selected":
                    self.includeViolations.append("\t"+feature+ "    |   " +inc+" [Status=False]")

    def excludes(self, feature, excludeList):
        mainExcFeatureStatus = self.findFeatureStatus(feature)
        if mainExcFeatureStatus == "selected":
            for exc in excludeList:
                statusCheck = self.findFeatureStatus(exc)
                if statusCheck == "selected":
                    self.excludeViolations.append("\t"+feature+ "    |   " +exc+" [Status=True]")

    

class ConstraintChecker(CrossFunctionalConstraints):
    def __init__(self):
        CrossFunctionalConstraints.__init__(self)

    def findFeatureMode(self, featureID):
        props = self.readProps()
        for prop in props['properties']:
            if prop['id'] == featureID:
                return prop['props']['mode']
    
    def parentChildMode(self):
        indexMaps = self.readIndexLookup()

        for mapping in indexMaps['mappings']:
            if mapping['parent'] != "root_bot":
                parentStatus = self.findFeatureStatus(mapping['parent'])
                childStatus = self.findFeatureStatus(mapping['child'])
                if parentStatus == "selected" and childStatus == "selected":
                    parentMode = self.findFeatureMode(mapping['parent'])
                    childMode = self.findFeatureMode(mapping['child'])
                    
                    if parentMode == "Dynamic" and childMode == "Static":
                        #append constraint violating feature
                        self.parentChildViolations.append([mapping['parent']+" (Dynamic)",mapping['child']+" (Static)"])

    

