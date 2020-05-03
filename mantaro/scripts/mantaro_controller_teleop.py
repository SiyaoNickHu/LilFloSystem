#!/usr/bin/env python

import rospy
import serial
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

MAX_SPD_PCT = 25

class MantaroController:
    def __init__(self, ser, fb_gain=60, lr_gain=30):
        self.ser = ser # serial object
        self.ser.write('M.C.0.0.0.0.0.0.\r')
        for i in range(4):
            rospy.sleep(0.5)
            self.ser.write('M.D.0.0.0.0.0.0.\r')
        self.prev_command_time = rospy.get_time()
        self.sub_base = rospy.Subscriber('/keyop_vel_smoother/raw_cmd_vel', Twist, self.base_callback)
        self.sub_pan_tilt = rospy.Subscriber('/keyop_vel_smoother/cam_raw_cmd_vel', Twist, self.pan_tilt_callback)
        self.battery_pub = rospy.Publisher('/mantaro_battery/voltage', Float32, queue_size=1)
        self.fb_gain = fb_gain
        self.lr_gain = lr_gain
        self.t = threading.Thread(target=self.battery_logger)
        self.t.daemon = True
        self.l = threading.Lock()
        self.t.start()

    def battery_logger(self):
        while 1:
            with self.l:
                self.ser.write('X.B.0.0.0.0.0.0.\r')
                data = self.ser.readline()
            if len(data) > 5:
                self.battery_pub.publish(float(data[1:5]) / 100.0)
            rospy.sleep(2)

    def base_callback(self, data):
        if rospy.get_time()-self.prev_command_time < 0.25:
            return

        if data.linear.x < 0:
            l_command = self.fb_gain * -data.linear.x
            r_command = self.fb_gain * -data.linear.x
            direction = 0
        else:
            l_command = min(max(0, self.fb_gain * data.linear.x - self.lr_gain * data.angular.z), MAX_SPD_PCT)
            r_command = min(max(0, self.fb_gain * data.linear.x + self.lr_gain * data.angular.z), MAX_SPD_PCT)
            direction = 1
        
        command = 'L.%d.%d.%d.R.%d.%d.%d.\r' % (direction, l_command/10, l_command%10, 
                                                direction, r_command/10, r_command%10)
        with self.l:
            self.ser.write(command)
            self.ser.readline()
        self.prev_command_time = rospy.get_time()

    def pan_tilt_callback(self, data):
        if rospy.get_time()-self.prev_command_time < 0.25:
            return

        if data.angular.z > 0:
            self.ser.write('M.E.0.0.0.0.0.0.\r')
        elif data.angular.z < 0:
            self.ser.write('M.I.0.0.0.0.0.0.\r')

        if data.angular.y > 0:
            self.ser.write('M.D.0.0.0.0.0.0.\r')
        elif data.angular.y < 0:
            self.ser.write('M.U.0.0.0.0.0.0.\r')

        with self.l:
            self.ser.readline()
        self.prev_command_time = rospy.get_time()


def run():
    rospy.init_node('mantaro_controller_teleop')

    with serial.Serial('/dev/Mantaro', 57600, xonxoff=True, timeout=0.5) as ser:
        rospy.sleep(2)

        ser.write('X.S.0.0.0.0.0.0.\r')

        controller = MantaroController(ser)
        rospy.spin()

if __name__ == '__main__':
    run()        