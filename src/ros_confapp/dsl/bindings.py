import rospy
from ros_confapp.dsl.dslState import DslState

class Bindings(DslState):
    def __init__(self):
        DslState.__init__(self)

    def getFeatureState(self, featureID):
        features = self.readProps()
        for feature in features['properties']:
            if feature['id'] == featureID:
                return feature['props']


    def getAppBindingTime(self):
        if rospy.get_param('/binding_time'):
            live_bt = rospy.get_param('/binding_time')
            return live_bt 
    
    def checkBindingCombination(self, featureID, action):
        featurePropsObject = self.getFeatureState(featureID)
        appBindingTime = self.getAppBindingTime()
        print(f'Current server binding time: {appBindingTime}')
        if action == "load":
            if appBindingTime == "late":
                #check if combo allows action
                if featurePropsObject['mode'].lower() == "static" and featurePropsObject['time'].lower() == "early":
                    return False
                elif featurePropsObject['mode'].lower() == "static" and featurePropsObject['time'].lower() == "late":
                    return True
                elif featurePropsObject['mode'].lower() == "dynamic" and featurePropsObject['time'].lower() == "early":
                    return True
                elif featurePropsObject['mode'].lower() == "dynamic" and featurePropsObject['time'].lower() == "late":
                    return True
            else:
                return True
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
                return True