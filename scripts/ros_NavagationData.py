import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped

class NavagationData:

    def __global_positionCallback(self,data):
        self.rocketLatitude = data.latitude
        self.rocketLongitude = data.longitude
        self.rocketAltitude = data.altitude
        print(self.rocketLatitude,self.rocketLongitude,self.rocketAltitude)

    def __local_positionCallback(self,data):
        self.rocketQuaternion = data.pose.orientation
        print('orientation',self.rocketQuaternion)

    def __init__(self):
        rospy.Subscriber('mavros/global_position/global',NavSatFix,self.__global_positionCallback)
        rospy.Subscriber('mavros/local_position/pose',PoseStamped,self.__local_positionCallback)
        
if __name__ == '__main__':
    rospy.init_node('tester',anonymous=True)
    navagationData=NavagationData()
    rospy.spin()