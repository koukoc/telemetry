#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool

class SystemState:

    def __firstStageIgnitionCallback(self,data):
        self.FirstStageIgnition = data.data

    def __RocketDetachmentCallback(self,data):
        self.RocketDetachment = data.data

    def __secondStageIgnitionCallback(self,data):
        self.SecondStageIgnition = data.data

    def __SeparationStateCallback(self,data):
        self.SeparationState = data.data

    def __OutofRackStateCallback(self,data):
        self.OutofRackState = data.data
    
    def __SeparateCommandCallback(self,data):
        self.Separate = data.data
    
    def __FirstStageMainValveCallback(self,data):
        self.FirstStageMainValve = data.data
    
    def __SecondStageMainValveCallback(self,data):
        self.SecondStageMainValve = data.data

    def __init__(self):
        rospy.Subscriber('FirstStageIgnition',Bool,self.__firstStageIgnitionCallback)
        rospy.Subscriber('SecondStageIgnition',Bool,self.__secondStageIgnitionCallback)
        rospy.Subscriber('RocketDetachment',Bool,self.__RocketDetachmentCallback)
        rospy.Subscriber('SeparationState',Bool,self.__SeparationStateCallback)
        rospy.Subscriber('OutofRackState',Bool,self.__OutofRackStateCallback)
        rospy.Subscriber('FirstStageMainValveOpened', Bool, self.__FirstStageMainValveCallback)
        rospy.Subscriber('SecondStageMainValveOpened', Bool, self.__SecondStageMainValveCallback)
        rospy.Subscriber('Separate', Bool, self.__SeparateCommandCallback)
        
if __name__ == '__main__':
    rospy.init_node('testersys',anonymous=True)
    sysstate=SystemState()
    rospy.spin()