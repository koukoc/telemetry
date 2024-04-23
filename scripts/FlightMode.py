import rospy
from std_msgs.msg import Bool

class FlightMode:

    def __HoldCallback(self,data):
        self.Hold = data.data
    
    def __LiftOffCallback(self,data):
        self.LiftOff = data.data
    
    def __MissionCallback(self,data):
        self.Mission = data.data

    def __SeperationCallback(self,data):
        self.Seperation = data.data

    def __init__(self):
        rospy.Subscriber('Hold',Bool,self.__HoldCallback)
        rospy.Subscriber('LiftOff',Bool,self.__LiftOffCallback)
        rospy.Subscriber('Mission',Bool,self.__MissionCallback)
        rospy.Subscriber('Seperation',Bool,self.__SeperationCallback)

if __name__ == '__main__':
    rospy.init_node('testersys',anonymous=True)
    flightstate=FlightMode()
    rospy.spin()