import rostopic
from ros_confapp.dsl.dslState import DslState

class Bindings(DslState):
    def __init__(self):
        DslState.__init__(self)

    def getFeatureState(self, featureID):
        features = self.readProps()
        for feature in features['properties']:
            if feature['id'] == featureID:
                return feature['props']

    def getAllBindings(self):
        bindings = []
        features = self.readProps()
        for feature in features['properties']:
            if feature['props']['type'].lower() != "abstract":
                bindings.append(feature['id'])
        return bindings


    def getAppBindingTime(self):
        try:
            rostopic.get_topic_class('/rosout')
            return "late"
        except rostopic.ROSTopicIOException as masterNotFound:
            return "early"
    
    def checkBindingCombination(self, featureID, action):
        featurePropsObject = self.getFeatureState(featureID)
        appBindingTime = self.getAppBindingTime()

        if action == "load":
            if appBindingTime == "late":
                #check if combo allows action
                if featurePropsObject['mode'].lower() == "static" and featurePropsObject['time'].lower() == "early":
                    return False
                elif featurePropsObject['mode'].lower() == "static" and featurePropsObject['time'].lower() == "late":
                    return True
                elif featurePropsObject['mode'].lower() == "dynamic" and featurePropsObject['time'].lower() == "early":
                    return False
                elif featurePropsObject['mode'].lower() == "dynamic" and featurePropsObject['time'].lower() == "late":
                    return True
            else:
                print("Node service temporarirly unavailable for load action. Restart Node.")
        elif action == "unload":
            if appBindingTime == "late":
                #check if combo allows action
                if featurePropsObject['mode'].lower() == "static" and featurePropsObject['time'].lower() == "early":
                    return False
                elif featurePropsObject['mode'].lower() == "static" and featurePropsObject['time'].lower() == "late":
                    return False
                elif featurePropsObject['mode'].lower() == "dynamic" and featurePropsObject['time'].lower() == "early":
                    return True
                elif featurePropsObject['mode'].lower() == "dynamic" and featurePropsObject['time'].lower() == "late":
                    return True
            else:
                print("Node service temporarily unavailable for unload action. Restart Node.")