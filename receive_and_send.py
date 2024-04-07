import struct
import serial
import serial.tools.list_ports
import time
import math

def get_data_from_CAN_bridge():
    byteArrayIncoming = bytearray()
    dataIncoming = None
    IDDummy = None
    cntr = 0
		
    if IDDummy == None:
        for i in range(1):	
            IDDummy = serial_connection.read()
        while dataIncoming == None:
            cntr+=1
            dataIncoming = serial_connection.read()
            if cntr == 1:
                ID = dataIncoming
            byteArrayIncoming.extend(dataIncoming) 
        
        for i in range(4):
            dataIncoming = serial_connection.read()
            if i != 0:
                byteArrayIncoming.extend(dataIncoming)
            else:
                continue
        
        # print(Bytesarray)
        data = struct.unpack('f', byteArrayIncoming)
        cntr = 0
        ID = ID[0]
        
        data = data[0]
        print(f"Received ID from CAN = {ID}")
        print(f"Received Data from CAN = {data}")
        print()
    

def pass_data_to_CAN_Bridge():
    idToSend = 42
    dataToSend = 19

    packet = bytearray()
    packet.append(idToSend)
    packet.append(dataToSend)
    

    val = 42069.214
    # while True:
    try:
        serial_connection.write(packet)
        print(f"The packet being sent is: {packet}")

    # try:
        ##serial_connection.write(binary_out)
        ##serial_connection.write(binary_data.encode('utf-8'))
        # sleep(1000)
        #serial_connection.write(packet)
        #print(f"The packet is: {packet}")
        
    except serial.SerialException as e:
        print("bruh: " + e)

    pass
    


serial_port = '/dev/ttyUSB2' # change back to 0 when USB0 works again

# serial_port = 'COM11'
baud_rate = 115200
serial_connection = serial.Serial(serial_port, baud_rate, timeout=1, bytesize=8)

cntr = 0
while True:
    try:
        get_data_from_CAN_bridge()
        # pass_data_to_CAN_Bridge()
        
    except serial.SerialException as e:
        print("BRUH: " + e)