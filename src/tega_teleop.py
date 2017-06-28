#!/usr/bin/env python

# Jacqueline Kory Westlund
# May 2016
#
# The MIT License (MIT)
#
# Copyright (c) 2016 Personal Robots Group
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import sys # exit and argv
import rospy # ROS
from PySide import QtGui, QtCore # basic GUI stuff
from r1d1_msgs.msg import TegaAction # ROS msgs
from r1d1_msgs.msg import TegaState # ROS msgs
from sar_opal_msgs.msg import OpalCommand # ROS msgs
from tega_teleop_ros import tega_teleop_ros
from tega_animation_ui import tega_animation_ui
from tega_lookat_ui import tega_lookat_ui
from tega_speech_ui import tega_speech_ui
from opal_tablet_ui import opal_tablet_ui
from tega_teleop_flags import tega_teleop_flags

class tega_teleop(QtGui.QMainWindow):
    """ Tega teleoperation interface """
    # set up ROS node globally
    # TODO if running on network where DNS does not resolve local
    # hostnames, get the public IP address of this machine and
    # export to the environment variable $ROS_IP to set the public
    # address of this node, so the user doesn't have to remember
    # to do this before starting the node.
    ros_node = rospy.init_node('tega_teleop', anonymous=True)

    def __init__(self):
        """ Initialize teleop interface """
        # setup GUI teleop interface
        super(tega_teleop, self).__init__()
        self.setGeometry(200,50,800,750)
        self.setWindowTitle("Tega Teleop")

        # create layout
        self.central_widget = QtGui.QWidget(self)
        self.central_layout = QtGui.QGridLayout(self.central_widget)
        self.setCentralWidget(self.central_widget)

        # add label for ROS messages to update
        self.ros_label = QtGui.QLabel(self)
        self.ros_label.setText("---")
        self.central_layout.addWidget(self.ros_label, 3, 6, 1, 1,
            alignment=QtCore.Qt.AlignLeft)

        # we have a boolean to flag whether child is attending or not based
        # on data coming in on the /child_attention topic from ROS
        # TODO this is a project-specific flag - need to revise how this
        # is done so that project-specific stuff can be swapped out for
        # new projects
        self.flags = tega_teleop_flags()

        # setup ROS node publisher and subscriber
        self.ros_teleop = tega_teleop_ros(self.ros_node, self.ros_label,
               self.flags)

        # add animation buttons
        anim_ui = tega_animation_ui(self.ros_teleop)
        self.central_layout.addWidget(anim_ui, 0, 0, 3, 7)

        # add lookat buttons
        lookat_ui = tega_lookat_ui(self.ros_teleop)
        self.central_layout.addWidget(lookat_ui, 2, 3, 2, 3)

        # add tablet controls
        opal_ui = opal_tablet_ui(self.ros_teleop)
        self.central_layout.addWidget(opal_ui, 2, 0, 2, 3)

        # add robot script playback buttons (mostly speech, but the scripts
        # can also list animations to play before or after an audio file)
        speech_ui = tega_speech_ui(self.ros_teleop, self.flags)
        self.central_layout.addWidget(speech_ui, 4, 0, 3, 7)

if __name__ == '__main__':
    # initialize top-level GUI manager
    app = QtGui.QApplication(sys.argv)

    # start teleop interface
    try:
        teleop_window = tega_teleop()
        teleop_window.show()

    # if roscore isn't running or shuts down unexpectedly
    except rospy.ROSInterruptException:
        print ('ROS node shutdown')
        pass

    # enter main loop, then exit
    sys.exit(app.exec_())
