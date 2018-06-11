import rospy
from serial_coms import SerialCom # https://github.com/Rehab-Robotics-Lab/serial_coms/tree/master/computer/python/serial_coms
import math
from flo_face.msg import FaceState


class FaceComs(object):
    """This class handles communicating with the face, given a state to send, it 
    will send it over the serial communciation line"""

    def __init__(self):
        rospy.init_node('face_com_manager')
        self.port = rospy.get_param('port', '/dev/ttyACM0')
        self.coms = SerialCom(self.port, self.data_handler)
        self.command_receipt = rospy.Subscriber('face_state',FaceState, self.new_command)

    def new_command(self,msg):
        self.coms.sendData([0] + bytize(msg.mouth))        
        self.coms.sendData([1] + bytize(msg.left_eye))
        self.coms.sendData([2] + bytize(msg.right_eye))        

    def bytize(flat_data):
        # flat_data = flatten(data)
        data_bytes = [0]*math.ceil(len(flat_data)/8)
        for i in range(len(data_bytes)):
            for j in range(8):
                data_bytes[i] = data_bytes[i] | (flat_data[i*8 + j] << (7-j))
        return data_bytes

    def data_handler(received, *data):
        rospy.logdebug("received as ints:")
        rospy.logdebug(data)
        rospy.logdebug("received as string (may be nonsense):")
        rospy.logdebug("".join(map(chr, data)))