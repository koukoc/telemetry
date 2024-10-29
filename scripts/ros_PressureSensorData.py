import rospy
from sensor_msgs.msg import UInt32


class PressureSensorData:

    def __FirstEnginePressureCallback(self,data):
        self.FirstEnginePressure = data.data
        
    def __FirstTankPressureCallback(self,data):
        self.FirstTankPressure = data.data

    def __SecondEnginePressureCallback(self,data):
        self.SecondEnginePressure = data.data

    def __SecondTankPressureCallback(self,data):
        self.SecondTankPressure = data.data

    def __RCSPressure1Callback(self,data):
        self.RCSPressure1 = data.data
    
    def __RCSPressure2Callback(self,data):
        self.RCSPressure2 = data.data
    
    def __RCSPressure3Callback(self,data):
        self.RCSPressure3 = data.data

    def __RCSPressure4Callback(self,data):
        self.RCSPressure4 = data.data
    
    def __RCSTankPressureCallback(self,data):
        self.RCSTankPressure = data.data

    def __init__(self):
        rospy.Subscriber('FirstEnginePressure',UInt32,self.__FirstEnginePressureCallback)
        rospy.Subscriber('FirstTankPressure',UInt32,self.__FirstTankPressureCallback)
        rospy.Subscriber('SecondEnginePressure',UInt32,self.__SecondEnginePressureCallback)
        rospy.Subscriber('SecondTankPressure',UInt32,self.__SecondTankPressureCallback)
        rospy.Subscriber('RCSPressure1',UInt32,self.__RCSPressure1Callback)
        rospy.Subscriber('RCSPressure2',UInt32,self.__RCSPressure2Callback)
        rospy.Subscriber('RCSPressure3',UInt32,self.__RCSPressure3Callback)
        rospy.Subscriber('RCSPressure4',UInt32,self.__RCSPressure4Callback)
        rospy.Subscriber('RCSTankPressure',UInt32,self.__RCSTankPressureCallback)



if __name__ == '__main__':
    rospy.init_node('tester',anonymous=True)
    pressureSensorData=PressureSensorData()
    rospy.spin()
