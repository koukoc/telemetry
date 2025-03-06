#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt32,Bool,UInt16
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped


class FCUSensor:
    def __init__(self):
        rospy.Subscriber('FirstBallValve',Bool,self.__FirstBallValveCallback)
        rospy.Subscriber('SecondBallValve',Bool,self.__SecondBallValveCallback)
        rospy.Subscriber('SeperationSwitch',Bool,self.__SecondSeparationSwitch1Callback)
        rospy.Subscriber('SolenoidValve1',Bool,self.__SolenoidValve1Callback) #TODO change to two call back function
        rospy.Subscriber('SolenoidValve2',Bool,self.__SolenoidValve2Callback)
        rospy.Subscriber('SolenoidValve3',Bool,self.__SolenoidValve3Callback)
        rospy.Subscriber('SolenoidValve4',Bool,self.__SolenoidValve4Callback)
        self.data={'FirstBallValve':False,'SecondBallValve':False,'SecondSeperationSwitch1':False,'SecondSeperationSwitch2':False,'SolenoidValve1':False,'SolenoidValve2':False,'SolenoidValve3':False,'SolenoidValve4':False,}

    def getData(self):
        return list(self.data.values())
    
    def __FirstBallValveCallback(self,data):
        self.data['FirstBallValve']=data.data

    def __SecondBallValveCallback(self,data):
        self.data['SecondBallValve']=data.data

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
        rospy.Subscriber('FirstEnginePressure',UInt32,self.__FirstEnginePressureCallback)
        rospy.Subscriber('FirstTankPressure',UInt32,self.__FirstTankPressureCallback)
        rospy.Subscriber('SecondEnginePressure',UInt32,self.__SecondEnginePressureCallback)
        rospy.Subscriber('SecondTankPressure',UInt32,self.__SecondTankPressureCallback)
        rospy.Subscriber('RCSPressure1',UInt32,self.__RCSPressure1Callback)
        rospy.Subscriber('RCSPressure2',UInt32,self.__RCSPressure2Callback)
        rospy.Subscriber('RCSPressure3',UInt32,self.__RCSPressure3Callback)
        rospy.Subscriber('RCSPressure4',UInt32,self.__RCSPressure4Callback)
        rospy.Subscriber('RCSTankPressure',UInt32,self.__RCSTankPressureCallback)
        self.data={'FirstEnginePressure':0,'FirstTankPressure':0,'SecondEnginePressure':0,'SecondTankPressure':0,'RCSPressure1':0,'RCSPressure2':0,'RCSPressure3':0,'RCSPressure4':0,'RCSTankPressure':0}

    def getData(self):
        return list(self.data.values())

    def __FirstEnginePressureCallback(self,data):
        self.data['FirstEnginePressure'] = data.data
        
    def __FirstTankPressureCallback(self,data):
        self.data['FirstTankPressure'] = data.data

    def __SecondEnginePressureCallback(self,data):
        self.data['SecondEnginePressure'] = data.data

    def __SecondTankPressureCallback(self,data):
        self.data['SecondTankPressure'] = data.data

    def __RCSPressure1Callback(self,data):
        self.data['RCSPressure1'] = data.data
    
    def __RCSPressure2Callback(self,data):
        self.data['RCSPressure2'] = data.data
    
    def __RCSPressure3Callback(self,data):
        self.data['RCSPressure3'] = data.data

    def __RCSPressure4Callback(self,data):
        self.data['RCSPressure4'] = data.data
    
    def __RCSTankPressureCallback(self,data):
        self.data['RCSTankPressure'] = data.data

    


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
        self.data['attitude']=[data.pose.orientation[i] for i in range(4)]

    def getData(self):
        return list(self.data.values())
# TODO fix Navagation Data
    



class SystemState:
    def __init__(self):
        rospy.Subscriber('FirstStageIgnition',Bool,self.__firstStageIgnitionCallback)
        rospy.Subscriber('SecondStageIgnition',Bool,self.__secondStageIgnitionCallback)
        rospy.Subscriber('RocketDetachment',Bool,self.__RocketDetachmentCallback)
        rospy.Subscriber('SeparationState',Bool,self.__SeparationStateCallback)
        rospy.Subscriber('OutofRackState',Bool,self.__OutofRackStateCallback)
        rospy.Subscriber('FirstStageMainValveOpened', Bool, self.__FirstStageMainValveCallback)
        rospy.Subscriber('SecondStageMainValveOpened', Bool, self.__SecondStageMainValveCallback)
        rospy.Subscriber('Separate', Bool, self.__SeparateCommandCallback)
        self.data={'FirstStageIgnition':False,'SecondStageIgnition':False,'RocketDetachment':False,'SeparationState':False,'OutofRackState':False,'FirstStageMainValveOpened':False,'SecondStageMainValveOpened':False,'Separate':False,}

    def __firstStageIgnitionCallback(self,data):
        self.data['FirstStageIgnition'] = data.data

    def __RocketDetachmentCallback(self,data):
        self.data['RocketDetachment'] = data.data

    def __secondStageIgnitionCallback(self,data):
        self.data['SecondStageIgnition'] = data.data

    def __SeparationStateCallback(self,data):
        self.data['SeparationState'] = data.data

    def __OutofRackStateCallback(self,data):
        self.data['OutofRackState'] = data.data
    
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

    