import rospy


class TVCData:

    def __TVCAngleCallback(self,data):
        self.TVCPitch = data.data
        self.TVCYaw = data.data
        self.TVCRoll = data.data

    def __init__(self):
        rospy.Subscriber('', ,self.__TVCAngleCallback)

if __name__ == '__main__':
    rospy.init_node('tester',anonymous=True)
    tvcData=TVCData()
    rospy.spin()