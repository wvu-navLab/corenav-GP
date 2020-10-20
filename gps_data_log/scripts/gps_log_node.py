#!/usr/bin/env python
import rospy
import serial
from datetime import datetime

def talker():
    ser = serial.Serial('/dev/serial/by-path/pci-0000:00:14.0-usb-0:3:1.0-port0', 115200, timeout=0)
    time_now = datetime.now()
    time_string = time_now.strftime("%Y-%m-%d-%H-%M-%S")
    gps_filename = time_string + "_gps.bin"
    timestamp_filename = time_string + "_ros_time.txt"
    gps_file = open(gps_filename,"w")
    timestamp_file = open(timestamp_filename,"w")
    rospy.init_node('gps_log_node', anonymous=False)
    rate = rospy.Rate(50) # Hz
    time_stamp_delta_time = 0.1 # sec
    prev_time = rospy.Time.now().to_sec() - time_stamp_delta_time
    rospy.loginfo("GPS Logger Running...")
    while not rospy.is_shutdown():
        data = ser.read(1000)
        #print(len(data))
        if len(data) > 0:
            ros_time = rospy.Time.now().to_sec()
            if ros_time - prev_time >= time_stamp_delta_time:
                ros_time_string = str(ros_time) + "\n"
                #print(ros_time_string)
                timestamp_file.write(ros_time_string)
                prev_time = ros_time
            gps_file.write(data)
        rate.sleep()
    gps_file.close()
    timestamp_file.close()
    ser.close()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
