#!/usr/bin/env python

import rospy
import serial
import time
import curses

def run(stdscr):
    rospy.init_node('mantaro_controller_keyboard')

    with serial.Serial('/dev/ttyUSB0', 57600, xonxoff=True, timeout=0.5) as ser:
        rospy.sleep(2)
        ser.write('X.P.0.0.0.0.0.0.\r')
        stdscr.addstr(ser.readline()+'\n\r')

        ser.write('X.B.0.0.0.0.0.0.\r')
        stdscr.addstr(ser.readline()+'\n\r')

        ser.write('X.S.0.0.0.0.0.0.\r')
        stdscr.addstr('starting board\n')

        prev_command_time = rospy.get_time()

        while True:
            c = stdscr.getch()

            if rospy.get_time()-prev_command_time < 0.25:
                continue

            if c != -1:
                c = chr(c)
                try:
                    if c == 'a': 
                        ser.write('L.0.0.0.R.1.0.5.\r')
                        stdscr.addstr('turn left\n\r')
                    elif c == 's': 
                        ser.write('L.0.0.5.R.0.0.5.\r')
                        stdscr.addstr('move backward\n\r')
                    elif c == 'd':
                        ser.write('L.1.0.5.R.0.0.0.\r')
                        stdscr.addstr('turn right\n\r')
                    elif c == 'w':
                        ser.write('L.1.0.5.R.1.0.5.\r')
                        stdscr.addstr('move forward\n\r')
                    elif c == 'q':
                        ser.write('X.C.0.0.0.0.0.0.\r')
                        return
                    elif c == 'j':
                        ser.write('M.E.0.0.0.0.0.0.\r')
                        stdscr.addstr('camera left\n')
                    elif c == 'k':
                        ser.write('M.D.0.0.0.0.0.0.\r')
                        stdscr.addstr('camera down\n')
                    elif c == 'l':
                        ser.write('M.I.0.0.0.0.0.0.\r')
                        stdscr.addstr('camera right\n')
                    elif c == 'i':
                        ser.write('M.U.0.0.0.0.0.0.\r')
                        stdscr.addstr('camera up\n')

                    prev_command_time = rospy.get_time()
                except:
                    stdscr.refresh()
                    stdscr.clear()
                    stdscr.move(0,0)



if __name__ == '__main__':
    curses.wrapper(run)