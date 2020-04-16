#!/usr/bin/env python

import rospy
import curses
from mantaro.srv import MantaroRequest, Mantaro
from geometry_msgs.msg import Twist

def run(stdscr):
    rospy.init_node('mantaro_fake_controller')
    pub_base = rospy.Publisher('/keyop_vel_smoother/raw_cmd_vel', Twist, queue_size=1)
    pub_cam = rospy.Publisher('/keyop_vel_smoother/cam_raw_cmd_vel', Twist, queue_size=1)
    # rospy.wait_for_service('mantaro_control')
    # mantaro_controller = rospy.ServiceProxy('mantaro_control', Mantaro)
    stdscr.nodelay(1)
    while True:
        c = stdscr.getch()
        if c != -1:
            c = chr(c)
            try:
                msg = Twist()
                if c == 'a': 
                    msg.angular.z = 1
                    pub_base.publish(msg)
                    stdscr.addstr('turn left\n\r')
                elif c == 's': 
                    msg.linear.x = -1
                    pub_base.publish(msg)
                    stdscr.addstr('move backward\n\r')
                elif c == 'd':
                    msg.angular.z = -1
                    pub_base.publish(msg)
                    stdscr.addstr('turn right\n\r')
                elif c == 'w':
                    msg.linear.x = 1
                    pub_base.publish(msg)
                    stdscr.addstr('move forward\n\r')
                elif c == 'q':
                    return
                elif c == 'j':
                    msg.angular.z = 1
                    pub_cam.publish(msg)
                    stdscr.addstr('camera left\n')
                elif c == 'k':
                    msg.angular.y = 1
                    pub_cam.publish(msg)
                    stdscr.addstr('camera down\n')
                elif c == 'l':
                    msg.angular.z = -1
                    pub_cam.publish(msg)
                    stdscr.addstr('camera right\n')
                elif c == 'i':
                    msg.angular.y = -1
                    pub_cam.publish(msg)
                    stdscr.addstr('camera up\n')
                # elif c == 'b':
                #     req = MantaroRequest()
                #     req.command = req.BATTERY
                #     resp = mantaro_controller(req)
                #     stdscr.addstr(str(resp.res)+'\n')
                # elif c == 'z':
                #     req = MantaroRequest()
                #     req.command = req.START
                #     resp = mantaro_controller(req)
                #     stdscr.addstr(str(resp.res)+'\n')
                # elif c == 'c':
                #     req = MantaroRequest()
                #     req.command = req.STOP
                #     resp = mantaro_controller(req)
                #     stdscr.addstr(str(resp.res)+'\n')
                # elif c == 'm':
                #     req = MantaroRequest()
                #     req.command = req.STATUS
                #     resp = mantaro_controller(req)
                #     stdscr.addstr(str(resp.res)+'\n')
            except:
                stdscr.refresh()
                stdscr.clear()
                stdscr.move(0,0)



if __name__ == '__main__':
    curses.wrapper(run)