#!/usr/bin/env python

import rospy
import serial
from geometry_msgs.msg import Twist
from mantaro.srv import Mantaro, MantaroResponse

MAX_SPD_PCT = 25

class MantaroController:
    def __init__(self, ser, fb_gain=60, lr_gain=30):
        self.ser = ser # serial object
        self.prev_command_time = rospy.get_time()
        self.sub_base = rospy.Subscriber('/keyop_vel_smoother/raw_cmd_vel', Twist, self.base_callback)
        self.sub_pan_tilt = rospy.Subscriber('/keyop_vel_smoother/cam_raw_cmd_vel', Twist, self.pan_tilt_callback)
        # self.server = rospy.Service('mantaro_control', Mantaro, self.mantaro_control)
        self.fb_gain = fb_gain
        self.lr_gain = lr_gain

    def mantaro_control(self, req):
        if req.command == req.START:
            self.ser.write('X.S.0.0.0.0.0.0.\r')
            print 'start'
            resp = MantaroResponse()
            resp.res = -1
            return resp
        elif req.command == req.STOP:
            self.ser.write('X.C.0.0.0.0.0.0.\r')
            print 'stop'
            resp = MantaroResponse()
            resp.res = -1
            return resp
        elif req.command == req.BATTERY:
            self.ser.write('X.B.0.0.0.0.0.0.\r')
            print 'battery'
            ret = self.ser.readline()
            resp = MantaroResponse()
            resp.res = float(ret[1:5])
            return resp
        elif req.command == req.STATUS:
            self.ser.write('X.P.0.0.0.0.0.0.\r')
            print 'status'
            ret = self.ser.readline()
            resp = MantaroResponse()
            if ret[0] == 'C':
                resp.res = resp.HALTED
            elif ret[0] == 'S':
                resp.res = resp.STARTED
            elif ret[0] == 'F':
                resp.res = resp.LOWVOLT
            return resp

    def base_callback(self, data):
        if rospy.get_time()-self.prev_command_time < 0.25:
            return

        # if data.angular.z > 0:
        #     l_command = self.fb_gain * data.linear.x + self.lr_gain * abs(data.angular.z) / 4
        #     r_command = self.fb_gain * data.linear.x + self.lr_gain * abs(data.angular.z) 
        # else:
        #     l_command = self.fb_gain * data.linear.x + self.lr_gain * abs(data.angular.z) 
        #     r_command = self.fb_gain * data.linear.x + self.lr_gain * abs(data.angular.z) / 4

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
        self.ser.write(command)
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

        self.prev_command_time = rospy.get_time()



def run():
    rospy.init_node('mantaro_controller_teleop')

    with serial.Serial('/dev/Mantaro', 57600, xonxoff=True, timeout=0.5) as ser:
        rospy.sleep(2)
        ser.write('X.P.0.0.0.0.0.0.\r')
        print ser.readline()

        ser.write('X.B.0.0.0.0.0.0.\r')
        print ser.readline()

        ser.write('X.S.0.0.0.0.0.0.\r')
        print 'starting board'

        controller = MantaroController(ser)
        rospy.spin()

if __name__ == '__main__':
    run()        