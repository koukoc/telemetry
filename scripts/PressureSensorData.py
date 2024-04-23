import rospy



class PressureSensorData:

    def __FirstPressureCallback(self,data):
        self.FirstEnginePressure = data.data
        self.FirstTankPressure = data.data

    def __SecondPressureCallback(self,data):
        self.SecondEnginePressure = data.data
        self.SecondTankPressure = data.data

    def __RCSPressureCallback(self,data):
        self.RCSPressure1 = data.data
        self.RCSPressure2 = data.data
        self.RCSPressure3 = data.data
        self.RCSPressure4 = data.data
        self.RCSTankPressure = data.data

    def __init__(self):
        rospy.Subscriber('', ,self.__FirstPressureCallback)
        rospy.Subscriber('', ,self.__SecondPressureCallback)
        rospy.Subscriber('', ,self.__RCSPressureCallback)


if __name__ == '__main__':
    rospy.init_node('tester',anonymous=True)
    pressureSensorData=PressureSensorData()
    rospy.spin()
