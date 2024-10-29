import rospy
from std_msgs.msg import Bool

class FlightMode:

    def __HoldCallback(self,data):
        self.HoldMode = data.data

    def __LiftOffCallback(self,data):
        self.HoldMode = data.data

    def __MissionCallback(self,data):
        self.HoldMode = data.data

    def __SeperationCallback(self,data):
        self.HoldMode = data.data

    def __init__(self):
        rospy.Subscriber('FlightMode',Bool,self.__HoldCallback)
        rospy.Subscriber('FlightMode',Bool,self.__LiftOffCallback)
        rospy.Subscriber('FlightMode',Bool,self.__MissionCallback)
        rospy.Subscriber('FlightMode',Bool,self.__SeperationCallback)
        
if __name__ == '__main__':
    rospy.init_node('testersys',anonymous=True)
    flightstate=FlightMode()
    rospy.spin()