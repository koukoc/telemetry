#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt32,Bool,UInt16,Float32
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped


class FCUSensor:
    def __init__(self):
        rospy.Subscriber('FirstStageMainValveState',Float32,self.__FirstStageMainValveStateCallback)
        rospy.Subscriber('SecoondStageMainValveState',Float32,self.__SecoondStageMainValveStateCallback)
        rospy.Subscriber('SeperationSwitch',Bool,self.__SecondSeparationSwitch1Callback)
        rospy.Subscriber('SolenoidValve1',Bool,self.__SolenoidValve1Callback) #TODO change to two call back function
        rospy.Subscriber('SolenoidValve2',Bool,self.__SolenoidValve2Callback)
        rospy.Subscriber('SolenoidValve3',Bool,self.__SolenoidValve3Callback)
        rospy.Subscriber('SolenoidValve4',Bool,self.__SolenoidValve4Callback)
        self.data={'FirstStageMainValveState':False,'SecoondStageMainValveState':False,'SecondSeperationSwitch1':False,'SecondSeperationSwitch2':False,'SolenoidValve1':False,'SolenoidValve2':False,'SolenoidValve3':False,'SolenoidValve4':False,}

    def getData(self):
        return list(self.data.values())
    
    def __FirstStageMainValveStateCallback(self,data):
        if data.data > 1.5:
            self.data['FirstStageMainValveState']=True
        else:
            self.data['FirstStageMainValveState']=False

    def __SecoondStageMainValveStateCallback(self,data):
        if data.data > 1.5:
            self.data['SecoondStageMainValveState']=True
        else:
            self.data['SecoondStageMainValveState']=False

    def __SecondSeparationSwitch1Callback(self,data):
        self.data['SeperationSwitch1']=data.data

    def __SolenoidValve1Callback(self,data):
        self.data['SolenoidValve1']=data.data

    def __SolenoidValve2Callback(self,data):
        self.data['SolenoidValve2']=data.data
    
    def __SolenoidValve3Callback(self,data):
        self.data['SolenoidValve3']=data.data

    def __SolenoidValve4Callback(self,data):
        self.data['SolenoidValve4']=data.data

    
class PressureSensorData:
    def __init__(self):
        print('start pressure sensor transmition') 
        rospy.Subscriber('FirstEnginePressure',Float32,self.__FirstEnginePressureCallback)
        rospy.Subscriber('FirstTankPressure',Float32,self.__FirstTankPressureCallback)
        rospy.Subscriber('SecondEnginePressure',Float32,self.__SecondEnginePressureCallback)
        rospy.Subscriber('SecondTankPressure',Float32,self.__SecondTankPressureCallback)
        rospy.Subscriber('RCSPressure1',Float32,self.__RCSPressure1Callback)
        rospy.Subscriber('RCSPressure2',Float32,self.__RCSPressure2Callback)
        rospy.Subscriber('RCSPressure3',Float32,self.__RCSPressure3Callback)
        rospy.Subscriber('RCSPressure4',Float32,self.__RCSPressure4Callback)
        rospy.Subscriber('RCSTankPressure',Float32,self.__RCSTankPressureCallback)
        self.FirstEnginePressure = 0.
        self.FirstTankPressure = 0.
        self.SecondEnginePressure = 0.
        self.SecondTankPressure = 0.
        self.RCSPressure1 = 0.
        self.RCSPressure2 = 0.
        self.RCSPressure3 = 0.
        self.RCSPressure4 = 0.
        self.RCSTankPressure = 0.

    # def getData(self):
    #     return [self.FirstEnginePressure,self.FirstTankPressure,self.SecondEnginePressure,self.SecondTankPressure,self.RCSPressure1,self.RCSPressure2,self.RCSPressure3,self.RCSPressure4,self.RCSTankPressure]

    def __FirstEnginePressureCallback(self,data):
        if data.data > 0:
            self.FirstEnginePressure = data.data
        else:
            self.FirstEnginePressure = 0
    def __FirstTankPressureCallback(self,data):
        if data.data > 0:
            self.FirstTankPressure = data.data
        else:
            self.FirstTankPressure = 0

    def __SecondEnginePressureCallback(self,data):
        if data.data > 0:
            self.SecondEnginePressure = data.data
        else:
            self.SecondEnginePressure = 0

    def __SecondTankPressureCallback(self,data):
        if data.data > 0:
            self.SecondTankPressure = data.data
        else:
            self.SecondTankPressure = 0

    def __RCSPressure1Callback(self,data):
        if data.data > 0:
            self.RCSPressure1 = data.data
        else:
            self.RCSPressure1 = 0
    
    def __RCSPressure2Callback(self,data):
        if data.data > 0:
            self.RCSPressure2 = data.data
        else:
            self.RCSPressure2 = 0
    
    def __RCSPressure3Callback(self,data):
        if data.data > 0:
            self.RCSPressure3 = data.data
        else:
            self.RCSPressure3 = 0

    def __RCSPressure4Callback(self,data):
        if data.data > 0:
            self.RCSPressure4 = data.data
        else:
            self.RCSPressure4 = 0
    
    def __RCSTankPressureCallback(self,data):
        if data.data > 0:
            self.RCSTankPressure = data.data
        else:
            self.RCSTankPressure = 0

    


class FlightMode:
    def __init__(self):
        rospy.Subscriber('FlightMode',UInt16,self.__FlightModeCallback)
        self.FlightModes = 0
    
    def __FlightModeCallback(self,data):
        self.FlightModes = data.data
    
    def getData(self):
        return self.FlightModes


class NavagationData:
    def __init__(self):
        rospy.Subscriber('mavros/global_position/global',NavSatFix,self.__global_positionCallback)
        rospy.Subscriber('mavros/local_position/pose',PoseStamped,self.__local_positionCallback)
        self.data={'position':[0,0,0],'attitude':[0,0,0,0]}
    
    def __global_positionCallback(self,data):
        self.data['position']=[data.latitude,data.longitude,data.altitude]

    def __local_positionCallback(self,data):
        self.data['attitude'] = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]

    def getData(self):
        return list(self.data.values())
# TODO fix Navagation Data
    



class SystemState:
    def __init__(self):
        rospy.Subscriber('FirstStageIgnite',Bool,self.__FirstStageIgniteCallback)
        rospy.Subscriber('leavetheRackState',Bool,self.__leavetheRackStateCallback)
        rospy.Subscriber('FirstStageMainValveOpened', Bool, self.__FirstStageMainValveCallback)
        rospy.Subscriber('Separate', Bool, self.__SeparateCommandCallback) # command to separate the rocket
        rospy.Subscriber('SeparationState',Bool,self.__SeparationStateCallback)  # show the state of separation
        rospy.Subscriber('SecondStageIgnite',Bool,self.__SecondStageIgniteCallback)
        rospy.Subscriber('SecondStageMainValveOpened', Bool, self.__SecondStageMainValveCallback)


        self.data={'FirstStageIgnite':False,'leavetheRackState':False
                   ,'FirstStageMainValveOpened':False,'Separate':False
                   ,'SeparationState':False,'SecondStageIgnite':False,'SecondStageMainValveOpened':False,}

    def __FirstStageIgniteCallback(self,data):
        self.data['FirstStageIgnite'] = data.data

    def __leavetheRackStateCallback(self,data):
        self.data['leavetheRackState'] = data.data

    def __SecondStageIgniteCallback(self,data):
        self.data['SecondStageIgnite'] = data.data

    def __SeparationStateCallback(self,data):
        self.data['SeparationState'] = data.data
    
    def __SeparateCommandCallback(self,data):
        self.data['Separate'] = data.data
    
    def __FirstStageMainValveCallback(self,data):
        self.data['FirstStageMainValve'] = data.data
    
    def __SecondStageMainValveCallback(self,data):
        self.data['SecondStageMainValve'] = data.data

    def getData(self):
        return list(self.data.values())


        
class TVCData:
    def __init__(self):
        rospy.Subscriber('TVCPitch',UInt16,self.__TVCPitchCallback)
        rospy.Subscriber('TVCYaw',UInt16,self.__TVCYawCallback)
        self.data={'TVCPitch':0,'TVCYaw':0} 
    
    def __TVCPitchCallback(self,data):
        self.data['TVCPitch'] = data.data

    def __TVCYawCallback(self,data):
        self.data['TVCYaw'] = data.data

    def getData(self):
        print(self.data)
        return list(self.data.values())    

    