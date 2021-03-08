Import("env", "projenv")
import serial;
import time;
import io

def after_upload(source, target, env):
    print("Start reset.")

    ser = serial.Serial("COM3", 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, xonxoff=0, rtscts=0)


    #if(ser.isOpen()):
    #    print("It was previously open so close it.")
    #    ser.close()

    if(not ser.isOpen()):
        ser.open()

    ser.setDTR(False)
    time.sleep(1)
    ser.flushInput()
    ser.setDTR(True)

    time.sleep(3)

    if(not ser.isOpen()):
        raise Exception("Could not open serial port.")

    print("Sending RST Command.")

    sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
    sio.write("RST")
    sio.flush()
    ser.write([0x0A])
    ser.flush()
    print("BTL Command Sent.")

    ser.close

env.AddPostAction("upload", after_upload)