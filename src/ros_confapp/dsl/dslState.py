#!/usr/bin/env python
import rospy
import os
import json

class DslState:
    def __init__(self):
        self.enginePath = os.path.dirname(os.path.abspath('../dsl/engineState.json'))
        self.lexPath = os.path.dirname(os.path.abspath('../dsl/langLexState.json'))
        self.regPath = os.path.dirname(os.path.abspath('../dsl/registry.json'))

    def getLexicalRules(self):
        lexStore = os.path.join(self.lexPath, 'langLexState.json')
        dataFile = open(lexStore, 'r')
        lexicons = json.loads(dataFile.read())
        dataFile.close()
        return lexicons

    def returnSingleLexRule(self, ruleName):
        lexicons = self.getLexicalRules()
        for lexicon  in lexicons['lexicons']:
            if lexicon['keyword'] == ruleName:
                return lexicon

    def getRegistryState(self):
        regStore = os.path.join(self.regPath, 'registry.json')
        dataFile = open(regStore, 'r')
        regEngineState = json.loads(dataFile.read())
        dataFile.close()
        return regEngineState
    
    def customGetRegistryState(self):
        rawReg = self.getRegistryState()
        
        indData = self.readIndexLookup()
        indIDs = []
        for element in indData['mappings']:
            indIDs.append(element['child'])

        customFeatureSet = []

        for vars in rawReg['env_var']:
            if vars['fid'] in indIDs:
                customFeatureSet.append(vars['fid'])
        
        return customFeatureSet
        

    def getEngineState(self):
        engineStore = os.path.join(self.enginePath, 'engineState.json')
        dataFile = open(engineStore, 'r')
        engineState = json.loads(dataFile.read())
        dataFile.close()
        return engineState

    def saveEngineState(self, engineData):
        engineStore = os.path.join(self.enginePath, 'engineState.json')
        with open(engineStore, 'w', encoding='utf-8') as f:
            json.dump(engineData, f, ensure_ascii=False, indent=4)


    def getCommandDetails(self, cmd):
        rules = self.getLexicalRules()
        for lexicon in rules['lexicons']:
            if cmd[0] == lexicon["keyword"]:
                return lexicon
        return False

    def readModel(self):
        eng = self.getEngineState()
        for proj in eng['projects']:
            if proj['status'] == 1:
                activeproj = proj['name']
                modelPath = os.path.dirname(os.path.abspath('../model/usr/'+activeproj+'/model.json'))
                my_model_file = os.path.join(modelPath, 'model.json')
                dataFile = open(my_model_file, 'r')
                readmodel = json.loads(dataFile.read())
                dataFile.close()
                return readmodel
            
        return False

    def readIndexLookup(self):
        eng = self.getEngineState()
        for proj in eng['projects']:
            if proj['status'] == 1:
                activeproj = proj['name']
                findexLookupPath = os.path.dirname(os.path.abspath('../model/usr/'+activeproj+'/index.json'))
                indfile = os.path.join(findexLookupPath, 'index.json')
                dataFile = open(indfile, 'r')
                index = json.loads(dataFile.read())
                dataFile.close()
                return index
        return False

    def readProps(self):
        eng = self.getEngineState()
        for proj in eng['projects']:
            if proj['status'] == 1:
                activeproj = proj['name']
                propsPath = os.path.dirname(os.path.abspath('../model/usr/'+activeproj+'/config.json'))
                propsfile = os.path.join(propsPath, 'config.json')
                dataFile = open(propsfile, 'r')
                allprops = json.loads(dataFile.read())
                dataFile.close()
                return allprops
        return False

    def readRegistry(self, fid):
        reg = self.getRegistryState()
        
        for vars in reg['env_var']:
            if vars['fid'] == fid:
                return vars['endpoint']
        return None


    def saveModel(self, modelData):
        eng = self.getEngineState()
        for proj in eng['projects']:
            if proj['status'] == 1:
                activeproj = proj['name']
                modelPath = os.path.dirname(os.path.abspath('../model/usr/'+activeproj+'/model.json'))
                modfile = os.path.join(modelPath, 'model.json')
                with open(modfile, 'w', encoding='utf-8') as f:
                    json.dump(modelData, f, ensure_ascii=False, indent=4)

    def saveIndexLookup(self, indexMappings):
        eng = self.getEngineState()
        for proj in eng['projects']:
            if proj['status'] == 1:
                activeproj = proj['name']
                luPath = os.path.dirname(os.path.abspath('../model/usr/'+activeproj+'/index.json'))
                lufile = os.path.join(luPath, 'index.json')
                with open(lufile, 'w', encoding='utf-8') as f:
                    json.dump(indexMappings, f, ensure_ascii=False, indent=4)

    def saveProps(self, properties):
        eng = self.getEngineState()
        for proj in eng['projects']:
            if proj['status'] == 1:
                activeproj = proj['name']
                propsPath = os.path.dirname(os.path.abspath('../model/usr/'+activeproj+'/config.json'))
                propsfile = os.path.join(propsPath, 'config.json')
                with open(propsfile, 'w', encoding='utf-8') as f:
                    json.dump(properties, f, ensure_ascii=False, indent=4)

    def verifyFeatureExistence(self, featureID):
        propList = self.readProps()
        for prop in propList['properties']:
            if prop['id'] == featureID:
                return True
        return False

    def dumpCurrentConfig(self):
        configDump = []
        propList = self.readProps()
        for prop in propList['properties']:
            if prop['props']['time'] == "Early" and prop['props']['status'] == True:
                configDump.append(prop['id'])     
        return configDump

    def setTimeToLate(self):
        engstate = self.getEngineState()
        for project in engstate['projects']:
            if project['status'] == 1:
                    project['time'] = "LATE"
                    rospy.loginfo("Runtime activated...")
        self.saveEngineState(engstate)

    def translateFeatureIDToNName(self, featureID):
        model = self.readModel()

            
                