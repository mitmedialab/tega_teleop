from PySide import QtGui # basic GUI stuff
import rospy # ROS
from r1d1_msgs.msg import TegaAction # ROS msgs
from r1d1_msgs.msg import TegaState # ROS msgs
from sar_opal_msgs.msg import OpalCommand # ROS msgs

class tega_teleop_ros():
    # ROS node
    # set up rostopics we publish
    tablet_pub = rospy.Publisher('opal_tablet_command', OpalCommand,
            queue_size = 10)
    tega_pub = rospy.Publisher('tega', TegaAction, queue_size = 10)

    def __init__(self, ros_node):
        """ Initialize ROS """
        self.ros_node = ros_node

    def send_opal_message(self, command):
        """ Publish opal command message """
        print('sending opal command')
        msg = OpalCommand()
        msg.command = command
        self.tablet_pub.publish(msg)
        rospy.loginfo(msg)

    def send_motion_message(self, motion):
        """ Publish TegaAction do motion message """
        print('sending motion message')
        msg = TegaAction()
        msg.do_motion = True
        msg.motion = motion
        self.tega_pub.publish(msg)
        rospy.loginfo(msg)

    def send_lookat_message(self, lookat):
        """ Publish TegaAction lookat message """
        print('sending lookat message')
        msg = TegaAction()
        msg.do_look_at = True
        msg.look_at = lookat
        self.tega_pub.publish(msg)
        rospy.loginfo(msg)

    def send_speech_message(self, speech):
        """ Publish TegaAction playback audio message """
        print ('\nsending speech message')
        msg = TegaAction()
        msg.do_sound_playback = True
        msg.wav_filename = speech
        self.tega_pub.publish(msg)
        rospy.loginfo(msg)

