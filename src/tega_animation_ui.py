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

from PySide import QtGui # basic GUI stuff
from r1d1_msgs.msg import TegaAction # ROS msgs
from tega_teleop_ros import tega_teleop_ros
from functools import partial

class tega_animation_ui(QtGui.QWidget):

   # list of animations for Tega
    animations = [
            TegaAction.MOTION_YES,
            TegaAction.MOTION_AGREEMENT,
            TegaAction.MOTION_LAUGH,
            TegaAction.MOTION_NO,
            TegaAction.MOTION_FRUSTRATED,
            TegaAction.MOTION_SAD,

            TegaAction.MOTION_NOD,
            TegaAction.MOTION_FLAT_AGREEMENT,
            TegaAction.MOTION_PERKUP,
            TegaAction.MOTION_PUZZLED,
            TegaAction.MOTION_SCARED,
            TegaAction.MOTION_SILENT_SAD,

            TegaAction.MOTION_EXCITED,
            TegaAction.MOTION_INTERESTED,
            TegaAction.MOTION_THINKING,
            TegaAction.MOTION_YAWN,
            TegaAction.MOTION_POSE1,
            TegaAction.MOTION_POSE2,

            TegaAction.MOTION_SHIMMY,
            TegaAction.MOTION_HAPPY_DANCE,
            TegaAction.MOTION_HAPPY_WIGGLE,
            TegaAction.MOTION_POSE_SLEEPING,
            TegaAction.MOTION_SHIFT_WEIGHT1,
            TegaAction.MOTION_SHIFT_WEIGHT2,

            TegaAction.MOTION_SWAY,
            TegaAction.MOTION_DANCE,
            TegaAction.MOTION_ROCKING,
            TegaAction.MOTION_POSE_FORWARD,
            TegaAction.MOTION_IDLESTILL,
            TegaAction.MOTION_SWIPE_STAGERIGHT
            ]

    def __init__(self, ros_node):
        """ Make a button for each animation """
        super(tega_animation_ui, self).__init__()
        # get reference to ros node so we can do callbacks to publish messages
        self.ros_node = ros_node

        # put buttons in a box
        anim_box = QtGui.QGroupBox(self)
        anim_layout = QtGui.QGridLayout(anim_box)
        anim_box.setTitle("Animations")

        # create animation buttons and add to layout
        col = 0
        row = 1
        for anim in self.animations:
            button = QtGui.QPushButton(anim.lower().replace("\"", ""), anim_box)
            button.clicked.connect(partial(self.ros_node.send_motion_message, anim))
            # if in the top left, make button green
            if (col < 3 and row < 4):
                button.setStyleSheet('QPushButton {color: green;}')
            # if in top right, make button red
            if (col > 2 and row < 3):
                button.setStyleSheet('QPushButton {color: red;}')
            anim_layout.addWidget(button, row, col)
            col += 1
            if(col >= 6): # six animation buttons per row
                col = 0
                row += 1
