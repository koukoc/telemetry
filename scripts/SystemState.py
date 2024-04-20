import rospy
from std_msgs.msg import Bool

class SystemState:

    def __firstStageIgnitionCallback(self,data):
        self.FirstStageIgnition = data.data

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
        rospy.Subscriber('SeparationState',Bool,self.__SeparationStateCallback)
        rospy.Subscriber('OutofRackState',Bool,self.__OutofRackStateCallback)
        rospy.Publisher('FirstStageMainValveOpened', Bool, self.__FirstStageMainValveCallback)
        rospy.Publisher('SecondStageMainValveOpened', Bool, self.__SecondStageMainValveCallback)
        rospy.Subscriber('Separate', Bool, self.__SeparateCommandCallback)
        