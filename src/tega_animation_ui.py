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

   # List of animations for Tega. Not all the SILENT animations are here.
    animations = [
            TegaAction.MOTION_YES,
            TegaAction.MOTION_INTERESTED,
            TegaAction.MOTION_SMILE,
            TegaAction.MOTION_POSE_SMILE1,
            TegaAction.MOTION_POSE_SMILE2,
            TegaAction.MOTION_NO,
            TegaAction.MOTION_SAD,
            TegaAction.MOTION_SLIGHTDISAPPROVEMENT,
            TegaAction.MOTION_DISAPPOINTMENT,
            TegaAction.MOTION_FROWN,

            TegaAction.MOTION_AGREEMENT,
            TegaAction.MOTION_FLAT_AGREEMENT,
            TegaAction.MOTION_LAUGH,
            TegaAction.MOTION_LAUGH_AGREEMENT,
            TegaAction.MOTION_LAUGH_YES,
            TegaAction.MOTION_FRUSTRATED,
            TegaAction.MOTION_SCARED,
            TegaAction.MOTION_SILENT_SAD,
            TegaAction.MOTION_SIGH,
            TegaAction.MOTION_SIGH1,

            TegaAction.MOTION_EXCITED,
            TegaAction.MOTION_LAUGHTER1,
            TegaAction.MOTION_LAUGHTER2,
            TegaAction.MOTION_LAUGHTER3,
            TegaAction.MOTION_LAUGHTER4,
            TegaAction.MOTION_THINKING,
            TegaAction.MOTION_ACTIVE_THINKING,
            TegaAction.MOTION_THINK1,
            TegaAction.MOTION_THINK2,
            TegaAction.MOTION_THINK3,

            TegaAction.MOTION_NOD,
            TegaAction.MOTION_MMHMMNOD,
            TegaAction.MOTION_EAGER1,
            TegaAction.MOTION_AHA,
            TegaAction.MOTION_PERKUP,
            TegaAction.MOTION_PUZZLED_EXCITED,
            TegaAction.MOTION_PUZZLED,
            TegaAction.MOTION_THINKINGEYES,
            TegaAction.MOTION_THINKINGFACES,
            TegaAction.MOTION_FART,

            TegaAction.MOTION_SLOWNOD,
            TegaAction.MOTION_NOD_TWICE,
            TegaAction.MOTION_PROUD,
            TegaAction.MOTION_BOUNCINGTILT,
            TegaAction.MOTION_CIRCLING,
            TegaAction.MOTION_BIG_NOD_NO_MOUTH,
            TegaAction.MOTION_UPDOWNLOOKING,
            TegaAction.MOTION_BlINK,
            TegaAction.MOTION_BLINKEDPOSED,
            TegaAction.MOTION_YAWN,

            TegaAction.MOTION_SHIMMY,
            TegaAction.MOTION_HAPPY_DANCE,
            TegaAction.MOTION_HAPPY_UP,
            TegaAction.MOTION_WIGGLE,
            TegaAction.MOTION_HAPPY_WIGGLE,
            TegaAction.MOTION_POSE_BREATHING,
            TegaAction.MOTION_POSE_BROW_RAISE,
            TegaAction.MOTION_POSE_LEFT,
            TegaAction.MOTION_POSE1,
            TegaAction.MOTION_POSE2,

            TegaAction.MOTION_CONFIRM,
            TegaAction.MOTION_SIDEPERK,
            TegaAction.MOTION_HEADPOINTBACK,
            TegaAction.MOTION_INTERESTED_ODDEYE,
            TegaAction.MOTION_LEFTRIGHTLOOKING,
            TegaAction.MOTION_POSE_FORWARD,
            TegaAction.MOTION_POSE_FORWARD_UP,
            TegaAction.MOTION_POSE_LEAN_FORWARD,
            TegaAction.MOTION_POSE_SLEEPING_SNORE,
            TegaAction.MOTION_POSE_SLEEPING,

            TegaAction.MOTION_SWAY,
            TegaAction.MOTION_DANCE,
            TegaAction.MOTION_DANCE1,
            TegaAction.MOTION_DANCE_BINGO,
            TegaAction.MOTION_ROCKING,
            TegaAction.MOTION_SWIPE_STAGERIGHT,
            TegaAction.MOTION_BROW_FURROW,
            TegaAction.MOTION_BROW_RAISE,
            TegaAction.MOTION_BROW_RAISE_IN,
            TegaAction.MOTION_BROW_RAISE_OUT
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
            if (col < 5 and row < 7):
                button.setStyleSheet('QPushButton {color: green;}')
            # if in top right, make button red
            if (col > 4 and row < 3):
                button.setStyleSheet('QPushButton {color: red;}')
            anim_layout.addWidget(button, row, col)
            col += 1
            if(col >= 10): # ten animation buttons per row
                col = 0
                row += 1
