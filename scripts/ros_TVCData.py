import rospy
from sensor_msgs.msg import Int16

class TVCData:

    def __TVCPitchCallback(self,data):
        self.TVCPitch = data.data

    def __TVCYawCallback(self,data):
        self.TVCYaw = data.data
    
    def __TVCRollCallback(self,data):
        self.TVCRoll = data.data

    def __init__(self):
        rospy.Subscriber('TVCPitch',Int16,self.__TVCPitchCallback)
        rospy.Subscriber('TVCYaw',Int16,self.__TVCYawCallback)
        rospy.Subscriber('TVCRoll',Int16,self.__TVCRollCallback)


if __name__ == '__main__':
    rospy.init_node('tester',anonymous=True)
    tvcData=TVCData()
    rospy.spin()