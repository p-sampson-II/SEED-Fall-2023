from smbus2 import SMBus, i2c_msg
from time import sleep
import struct
from array import array

ARD_ADDR = 8
mech = SMBus(1)

instr = {
    "turn":0,
    "fwd":1,
    "datareq":2,
    "curve":3
}

reqLen = {
    "byte" : 1,
    "word": 4,
    "double" : 8,
    "max" : 32
}

reqSelect = {
    "phi" : 0,
    "xy" : 1
}

selLength = {
    0 : 4,
    1 : 8
}
def menu(options : dict) -> int:
    for ind,option in enumerate(options):
        print("%i : %s"%(ind,option))
    selection = requestNumber("Enter Selection:",int)
    return selection

def requestNumber(prompt : str, retType : object):
    output = 0
    while True:
        userIn = input(prompt)
        try:
            output = retType(userIn)
        except ValueError:
            print("Enter a number!")
            continue
        else:
            break
    return output
    

def selectorTree(selection : int):
    if selection == instr["turn"] or selection == instr["fwd"]:
        value = requestNumber("Enter the value: ", float)
        message = list(struct.pack('<f',value))
        try:
            sendFollower(selection,message)
        except TypeError:
            print("The data was of the wrong type: %s"%str(type(message)))
        return 0
    if selection == instr["datareq"]:
        reqType = requestNumber("Enter the request type: ", str)
        return receiveFollower(selection,reqSelect[reqType])
        
def receiveFollower(offset,reqType):
    dataIn = []
    try:
        sendFollower(offset,reqType)
    except TypeError:
        print("The data was of the wrong type: %s"%str(type(message)))
    sleep(0.2)
    try:
        dataIn = mech.read_i2c_block_data(ARD_ADDR,offset,selLength[reqType])
    except OSError:
        print("Cannot retrieve data from mech. system") 
    print(dataIn)
    if selLength[reqType] == 4:
        myFloat = struct.unpack('<f',bytes(dataIn))
    if selLength[reqType] == 8:
        myFloat = struct.unpack('<ff',bytes(dataIn))
    print(myFloat)
    
        
        
def sendFollower(offset,data):
    if type(data) == int:
        data = [data]
    if type(data) == list:
        try:
            mech.write_i2c_block_data(ARD_ADDR,offset,data)
        except OSError:
            print("Cannot reach mechatronic controller.")
    else:
        raise TypeError()

def waitForCompletion():
    mechOut = 0
    while(not mechOut):
        sleep(0.1)
        try:
            mechOut = mech.read_byte_data(ARD_ADDR,0)
        except OSError:
            print("No ack/nack from Arduino")
            break

while True:
    select = menu(instr)
    selectorTree(select)
    waitForCompletion()
