import rospy
from std_msgs.msg import Bool

class FCUSensor:

    def __FirstBallValveCallback(self,data):
        self.FirstBallValve = data.data

    def __SecondSeperationSwitch1Callback(self,data):
        self.SecondSeperationSwitch1 = data.data

    def __SecondSeperationSwitch2Callback(self,data):
        self.SecondSeperationSwitch2 = data.data

    def __SecondBallValveCallback(self,data):
        self.SecondBallValve = data.data

    def __SolenoidValve1Callback(self,data):
        self.SolenoidValve1 = data.data

    def __SolenoidValve2Callback(self,data):
        self.SolenoidValve2 = data.data
    
    def __SolenoidValve3Callback(self,data):
        self.SolenoidValve3 = data.data

    def __SolenoidValve4Callback(self,data):
        self.SolenoidValve4 = data.data

    def __init__(self):
        rospy.Subscriber('FirstBallValve',Bool,self.__FirstBallValveCallback)
        rospy.Subscriber('SecondSeperationSwitch1',Bool,self.__SecondSeperationSwitch1Callback)
        rospy.Subscriber('SecondSeperationSwitch2',Bool,self.__SecondSeperationSwitch2Callback)
        rospy.Subscriber('SecondBallValve',Bool,self.__SecondBallValveCallback)
        rospy.Subscriber('SolenoidValve1',Bool,self.__SolenoidValve1Callback)
        rospy.Subscriber('SolenoidValve2',Bool,self.__SolenoidValve2Callback)
        rospy.Subscriber('SolenoidValve3',Bool,self.__SolenoidValve3Callback)
        rospy.Subscriber('SolenoidValve4',Bool,self.__SolenoidValve4Callback)

if __name__ == '__main__':
    rospy.init_node('testersys',anonymous=True)
    fcustate=FCUSensor()
    rospy.spin()
