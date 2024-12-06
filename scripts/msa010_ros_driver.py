#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

import serial
import numpy as np
import cv2
import json
import time


BAUD = {0: 9600, 1: 57600, 2: 115200, 3: 230400, 4: 460800, 5: 921600, 6: 1000000, 7: 2000000, 8: 3000000}
BAUD_MAXBOT = {2: 115200, 5: 921600}


class msa010Driver:
    def __init__(self):
        self.device = rospy.get_param("msa010_ros_driver/device", "/dev/depth_camera")
        self.frame_id = rospy.get_param("msa010_ros_driver/frame_id", "dep_cam_front_link")

        self.depth_img_pub = rospy.Publisher("depth/image_raw", Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher("depth/camera_info", CameraInfo, queue_size=1)

        self.packet_start_time = 0.0
        self.packet_timeout = 1.0

        self.ser = serial.Serial()

        self.ser.port = self.device
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.xonxoff = False
        self.ser.rtscts = False
        self.ser.dsrdtr = False
        self.ser.timeout = 0.2
        self.ser.write_timeout = 0.2
        # self.ser.inter_byte_timeout = 
        # self.ser.exclusive = 

        # connect to CP2102 module and msa010 device
        while True:
            if self.connectDevice():
                break

        self.setSettings(baud_value=5)

        self.fx, self.fy, self.u0, self.v0 = self.intrinsicParam()
        # self.fx, self.fy, self.u0, self.v0 = 75, 75, 50, 50

        self.setSettings(isp_value=1, binn_value=1, unit_value=0, fps_value=10, antimmi_value=-1)
        print("Serial Initialization Completed.")

        self.setSettings(disp_value=4)
        print("Start Receiving Depth Image.")

        self.bridge = CvBridge()

        self.header = Header()
        self.header.frame_id = self.frame_id

        self.cam_info = CameraInfo()
        self.cam_info.height = 100
        self.cam_info.width = 100
        self.cam_info.distortion_model = "plumb_bob"
        self.cam_info.D = [0, 0, 0, 0, 0]
        self.cam_info.K = [self.fx, 0, self.u0, 0, self.fy, self.v0, 0, 0, 1]
        self.cam_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        self.cam_info.P = [self.fx, 0, self.u0, 0, 0, self.fy, self.v0, 0, 0, 0, 1, 0]

        self.msa010Publisher()


    def connectDevice(self):
        for key, baudrate in BAUD_MAXBOT.items():
            try:
                if not self.ser.is_open:
                    self.ser.open()
                    print("Successfully opened the serial port.")
                command = "AT\r"
                self.ser.write(command.encode("ASCII"))
                response = self.ser.readline()
                if b"OK\r\n" in response:
                    print("Received resopnse from msa010 at baud ", self.ser.baudrate)
                    return True
                else:
                    for i in range(10):
                        command = "AT+BAUD=%1d\r" % key
                        self.ser.write(command.encode("ASCII"))
                        response = self.ser.readline()
                        self.ser.close()
                        self.ser.baudrate = BAUD_MAXBOT[key]
                        self.ser.open()
                        if b"OK\r\n" in response: 
                            print("Received resopnse from msa010 at baud ", self.ser.baudrate)
                            return True
                        else:
                            print("Not received resopnse from msa010 at baud ", self.ser.baudrate)

            except Exception as e:
                print("Serial error: ", e)
                try:
                    if self.ser.is_open:
                        self.ser.close()
                        print("Serial port closed due to an error.")
                except Exception as close_error:
                    print("Error during serial port close: ", close_error)

                rospy.sleep(1)

                try:
                    if not self.ser.is_open:
                        self.ser.open()
                        print("Serial port reconnected.")
                except Exception as open_error:
                    print("Failed to reopen the serial port: ", open_error)

                rospy.sleep(1)

        print("Failed to connect device.")
        return False


    def intrinsicParam(self):
        while True:
            try:
                command = "AT+COEFF?\r"
                self.ser.write(command.encode("ASCII"))
                response = self.ser.readlines()
                if b"OK\r\n" in response:
                    print("COEFF Response: ", response)
                    response_string = b''.join(response).decode()
                    response_string = response_string.strip()
                    response_string = '\n'.join(response_string.split('\n')[2:])
                    cparms = json.loads(response_string)
                    fx = cparms["fx"] / 262144
                    fy = cparms["fy"] / 262144
                    u0 = cparms["u0"] / 262144
                    v0 = cparms["v0"] / 262144
                    print(fx, fy, u0, v0)
                    print("------------------------------")
                    return fx, fy, u0, v0
                else:
                    continue
            except Exception as e:
                print("Serial error when read intrinsic param: ", e)
                rospy.sleep(1)


    def printSettings(self):
        while True:
            try:
                command = "AT+ISP?\r"
                self.ser.write(command.encode("ASCII"))
                response = self.ser.readlines()
                if b"OK\r\n" in response:
                    print("ISP Response: ", response)
                else:
                    continue
                command = "AT+BINN?\r"
                self.ser.write(command.encode("ASCII"))
                response = self.ser.readlines()
                if b"OK\r\n" in response:
                    print("BINN Response: ", response)
                else:
                    continue
                command = "AT+DISP?\r"
                self.ser.write(command.encode("ASCII"))
                response = self.ser.readlines()
                if b"OK\r\n" in response:
                    print("DISP Response: ", response)
                else:
                    continue
                command = "AT+BAUD?\r"
                self.ser.write(command.encode("ASCII"))
                response = self.ser.readlines()
                if b"OK\r\n" in response:
                    print("BAUD Response: ", response)
                else:
                    continue
                command = "AT+UNIT?\r"
                self.ser.write(command.encode("ASCII"))
                response = self.ser.readlines()
                if b"OK\r\n" in response:
                    print("UNIT Response: ", response)
                else:
                    continue
                command = "AT+FPS?\r"
                self.ser.write(command.encode("ASCII"))
                response = self.ser.readlines()
                if b"OK\r\n" in response:
                    print("FPS Response: ", response)
                else:
                    continue
                command = "AT+ANTIMMI?\r"
                self.ser.write(command.encode("ASCII"))
                response = self.ser.readlines()
                if b"OK\r\n" in response:
                    print("ANTIMMI Response: ", response)
                else:
                    continue
                print("------------------------------")
                return
            except Exception as e:
                print("Serial error when print settings: ", e)
                rospy.sleep(1)


    """Set TOF sensor parameters
    Inputs:
        ISP: 0: turn ISP off; 1: turn ISP on
        BINN: 1: output 100x100 pixel frame; 2: output 50x50 pixel frame; 4: output 25x25 pixel frame
        DISP: 0: all off; 1: lcd display on; 2: usb display on; 3: lcd and usb display on; 4: uart display on; 
              5: lcd and uart display on; 6: usb and uart display on; 7: lcd, usb and uart display on
        BAUD: 0: 9600; 1: 57600; 2: 115200; 3: 230400; 4: 460800; 5: 921600; 6: 1000000; 7: 2000000; 8: 3000000
        UNIT: 0: auto; 1-10: quantizated by unit(mm)
        FPS: 1-19: set frame per second
        ANTIMMI: -1: disable anti-mmi; 0: auto anti-mmi; 1-41: manual anti-mmi usb display on

    Return:
        Set parameters through serial
    """
    def setSettings(self, isp_value=None, binn_value=None, disp_value=None, baud_value=None, 
                          unit_value=None, fps_value=None, antimmi_value=None):
        while True:
            try:
                if isp_value is not None: 
                    command = "AT+ISP=%1d\r" % isp_value
                    self.ser.write(command.encode("ASCII"))
                    response = self.ser.readline()
                    if b"OK\r\n" in response:
                        print("ISP Response: ", response)
                    else:
                        continue
                if binn_value is not None: 
                    command = "AT+BINN=%1d\r" % binn_value
                    self.ser.write(command.encode("ASCII"))
                    response = self.ser.readline()
                    if b"OK\r\n" in response:
                        print("BINN Response: ", response)
                    else:
                        continue
                if disp_value is not None: 
                    command = "AT+DISP=%1d\r" % disp_value
                    self.ser.write(command.encode("ASCII"))
                    response = self.ser.readline()
                    if b"OK\r\n" in response: 
                        print("DISP Response: ", response)
                    else:
                        continue
                if baud_value is not None: 
                    command = "AT+BAUD=%1d\r" % baud_value
                    self.ser.write(command.encode("ASCII"))
                    response = self.ser.readline()
                    self.ser.close()
                    self.ser.baudrate = BAUD[baud_value]         # change the baudrate of the serial 
                    self.ser.open()
                    if b"OK\r\n" in response: 
                        print("BAUD Response: ", response)
                    else:
                        continue
                if unit_value is not None: 
                    command = "AT+UNIT=%1d\r" % unit_value
                    self.ser.write(command.encode("ASCII"))
                    response = self.ser.readline()
                    if b"OK\r\n" in response:
                        print("UNIT Response: ", response)
                    else:
                        continue
                if fps_value is not None: 
                    command = "AT+FPS=%1d\r" % fps_value
                    self.ser.write(command.encode("ASCII"))
                    response = self.ser.readline()
                    if b"OK\r\n" in response:
                        print("FPS Response: ", response)
                    else:
                        continue
                if antimmi_value is not None:
                    command = "AT+ANTIMMI=%1d\r" % antimmi_value
                    self.ser.write(command.encode("ASCII"))
                    response = self.ser.readline()
                    if b"OK\r\n" in response:
                        print("ANTIMMI Response: ", response)
                    else:
                        continue
                print("------------------------------")
                return
            except Exception as e:
                print("Serial error when set settings: ", e)
                rospy.sleep(1)


    def displayImage(self, frame_data):
        cv2.imshow("Depth Image", frame_data)
        cv2.waitKey(1)


    def getCurrentTime(self):
        return round(time.time() * 1000000000) / 1000000.0 


    def getTimeSinceStart(self):
        time_since = self.getCurrentTime() - self.packet_start_time
        if time_since < 0.0:
            self.packet_start_time = self.getCurrentTime()

        return time_since


    def isPacketTimeout(self):
        if self.getTimeSinceStart() > self.packet_timeout:
            self.packet_timeout = 0
            return True

        return False


    def rxPacket(self):
        rxpacket = bytearray()

        checksum = 0
        rx_length = 0
        wait_length = 10022

        success = False

        while True:
            try:
                rxpacket.extend(self.ser.read(wait_length - rx_length))
                rx_length = len(rxpacket)

                if rx_length >= wait_length:
                    # find packet header 
                    for idx in range(0, (rx_length - 1)):
                        if (rxpacket[idx] == 0) and (rxpacket[idx + 1] == 255):
                            break

                    if idx == 0:
                        if rx_length < wait_length:
                            # check timeout
                            if self.isPacketTimeout():
                                success = False
                                break
                            else:
                                continue

                        # calculate checksum
                        for byte in rxpacket[:-2]: 
                            checksum += byte
                        checksum &= 0xFF

                        # verify checksum and end
                        if rxpacket[-2] == checksum and rxpacket[-1] == 221:
                            success = True
                        else:
                            success = False 
                        break
                    else:
                        del rxpacket[0:idx]
                        rx_length -= idx
                else:
                    if self.isPacketTimeout():
                        success = False
                        break

            except Exception as e:
                print("Serial error when receive packet: ", e)
                success = False
                break
        
        return rxpacket, success


    def msa010Publisher(self):
        while not rospy.is_shutdown():
            try:
                self.packet_start_time = self.getCurrentTime()
                rxpacket, success = self.rxPacket()
                if success:
                    img = np.frombuffer(rxpacket[20:10020], dtype=np.uint8)
                    img = np.reshape(img, (self.cam_info.height, self.cam_info.width))

                    # header 
                    self.header.stamp = rospy.Time.now()

                    # camera info
                    self.cam_info.header = self.header
                    self.camera_info_pub.publish(self.cam_info)

                    # depth image 
                    img_msg = self.bridge.cv2_to_imgmsg(img, encoding="8UC1")
                    img_msg.header = self.header
                    self.depth_img_pub.publish(img_msg)
                else:
                    rospy.logwarn("Failed to receive a complete frame.")
                    print("Trying to reconnect.")
                    if rospy.is_shutdown():
                        print("ROS shutdown.")
                        break
                    elif self.connectDevice():
                        rospy.logwarn("Reconnect finished.")
                        self.setSettings(baud_value=5)
                        self.setSettings(isp_value=1, binn_value=1, unit_value=0, fps_value=10, antimmi_value=-1)
                        print("Serial Initialization Completed.")
                        self.setSettings(disp_value=4)
                        print("Start Receiving Depth Image.")

            except Exception as e:
                print("Serial error in main loop: ", e)

        self.setSettings(disp_value=0)
        self.setSettings(baud_value=2)
        print("Serial Closing.")
        self.ser.close()


if __name__ == '__main__':
    try:
        rospy.init_node('msa010_driver', anonymous=True)
        msa010_driver = msa010Driver()
    except rospy.ROSInterruptException:
        pass