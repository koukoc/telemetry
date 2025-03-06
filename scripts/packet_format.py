packet = {
    'System State Struture':{
        'Header' : b'\x01',
        'data length': 7,
        'structure': [
            ('Header'   , 'byte', 1),
            ('Time'     , 'byte', 4),
            ('Info'     , 'bool', 1),
            ('Checksum' , 'byte', 1)
        ],
        'Info' : [
            ('Stage1IgnitionSignal'),
            ('DetachmentSignal'),
            ('Stage1MainValveSignal'),
            ('Stage1SeparateSignal'),
            ('Stage1SeparateSensorSignal'),
            ('Stage2IgnitionSignal'),
            ('Stage2MainValveSignal')
            #('Dummy')
        ]
    },

    'Flight mode Struture':{
        'Header' : b'\x02',
        'data length': 7,
        'structure': [
            ('Header'   , 'byte', 1),
            ('Time'     , 'byte', 4),
            ('Info'     , 'bool', 1),
            ('Checksum' , 'byte', 1)
        ],
        'Info':[
            ('FlightMode')
            #('FlightMode'),
            #('Dummy'),
            #('Dummy'),
            #('Dummy'),
            #('Dummy'),
            #('Dummy'),
            #('Dummy')
        ]
    },

    'FCU State Struture':{
        'Header' : b'\x03',
        'data length': 7,
        'structure':[
            ('Header'   , 'byte', 1),
            ('Time'     , 'byte', 4),
            ('Info'     , 'bool', 1),
            ('Checksum' , 'byte', 1)
        ],
        'Info':[
            ('S1BallValve'),
            ('SeperationSwitch'),
            ('S2BallValve'),
            ('SolenoidValve1'),
            ('SolenoidValve2')
            #('Dummy'),
            #('Dummy'),
            #('Dummy'),
        ]
    },

    'Navigation Data Struture':{
        'Header' : b'\x04',
        'data length': 34,
        'structure':[
            ('Header'   , 'byte', 1),
            ('Time'     , 'byte', 4),
            ('Info'     , 'float', 28),
            ('Checksum' , 'byte', 1)
        ],
        'Info': [
            ('QUAT_R'),
            ('QUAT_I'),
            ('QUAT_J'),
            ('QUAT_K'),
            ('GPS_LAT'),
            ('GPS_LONG'),
            ('GPS_ALT')
        ]
    },

    'TVC Data Struture':{
        'Header' : b'\x05',
        'data length': 12,
        'structure':[
            ('Header'   , 'byte', 1),
            ('Time'     , 'byte', 4),
            ('Info'     , 'short', 6),
            ('Checksum' , 'byte', 1)
        ],
        'Info': [
            ('PITCH_TRUE'),
            ('YAW_TRUE'),
            ('ROLL_SP')
        ]
    },

    'Pressure Data Struture':{
        'Header' : b'\x06',
        'data length': 24,
        'structure':[
            ('Header'   , 'byte', 1),
            ('Time'     , 'byte', 4),
            ('Info'     , 'int', 18),
            ('Checksum' , 'byte', 1)
        ],
        'Info': [
            ('FirstEnginePressure'),
            ('FirstTankPressure'),
            ('SecondEnginePressure'),
            ('SecondTankPressure'),
            ('RCSPressure1'),
            ('RCSPressure2'),
            ('RCSPressure3'),
            ('RCSPressure4'),
            ('RCSTankPressure')
        ]
    }
}

#header_to_packet = {}
#for packet_name, definition in packet.items():
#    
#    header = definition['Header']
#    header_to_packet[header] = definition
#
#print (header_to_packet,'\n')
#
#



