#!/usr/bin/env python

import sys # exit and argv
import rospy # ROS
from PySide import QtGui, QtCore # basic GUI stuff
from r1d1_msgs.msg import TegaAction # ROS msgs
from r1d1_msgs.msg import TegaState # ROS msgs
from sar_opal_msgs.msg import OpalCommand # ROS msgs
from tega_animation_ui import tega_animation_ui

class tega_teleop(QtGui.QMainWindow):
    """ Tega teleoperation interface """
     # ROS node
    # start ROS node and list topics we publish
    rospy.init_node('tega_teleop', anonymous=True)
    tablet_pub = rospy.Publisher('opal_tablet_command', OpalCommand, queue_size = 10)
    tega_pub = rospy.Publisher('tega_action', TegaAction, queue_size = 10)
    r = rospy.Rate(10) # spin at 10 Hz
    #r.sleep() # sleep to wait for subscribers

    def __init__(self):
        """ Initialize teleop interface """
        # setup GUI teleop interface
        super(tega_teleop, self).__init__()
        self.setGeometry(100,100,600,300)
        self.setWindowTitle("Tega Teleop")

        # create layout 
        self.central_widget = QtGui.QWidget(self)
        self.central_layout = QtGui.QGridLayout(self.central_widget)
        self.setCentralWidget(self.central_widget)

        # add animation buttons
        anim_ui = tega_animation_ui()
        self.central_layout.addWidget(anim_ui, 0, 0)

        #TODO add other button collections in the grid too!
      
        # start ROS node
        # do that here?


                

    def setupLookatButtons(self):
        """ Make buttons to look different directions """
        # label the section
        
        # look left
        # TODO calibrate lookat
        #self.button = tk.Button(self.lookat_frame, 
                    #text="left",
                    #command=lambda: self.send_lookat_message((10,0,0)))
        #self.button.grid(column=1, row=6, sticky=tk.W)

        # look center
        #self.button = tk.Button(self.lookat_frame, 
                    #text="center",
                    #command=lambda: self.send_lookat_message((0,0,0)))
        #self.button.grid(column=2, row=6, sticky=tk.W)
       
        # look right
        #self.button = tk.Button(self.lookat_frame, 
                    #text="right",
                    #command=lambda: self.send_lookat_message((-10,0,0)))
        #self.button.grid(column=3, row=6, sticky=tk.W)

        # look down
        #self.button = tk.Button(self.lookat_frame, 
                    #text="down",
                    #command=lambda: self.send_lookat_message((0,-10,0)))
        #self.button.grid(column=4, row=6, sticky=tk.W)
       
        # look up
        #self.button = tk.Button(self.lookat_frame, 
                    #text="up",
                    #command=lambda: self.send_lookat_message((0,10,0)))
        #self.button.grid(column=5, row=6, sticky=tk.W)

        
    #def setupSpeechButtons(self):
        """ Make buttons to trigger speech """
                

    def send_opal_message(self, command):
        """ Publish opal command message """
        print('sending opal command')
        msg = OpalCommand()
        msg.command = command
        self.tablet_pub.publish(msg)
        rospy.loginfo(msg)


# TODO combine these, one function with if-elses to pack msg?
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


if __name__ == '__main__':
    # initialize top-level GUI manager
    app = QtGui.QApplication(sys.argv)

    # start teleop interface
    #try:
    teleop_window = tega_teleop()
    teleop_window.show()

    #except rospy.ROSInterruptException: 
        #print ('ROSnode shutdown')
        #pass

    # enter main loop, then exit
    sys.exit(app.exec_())
