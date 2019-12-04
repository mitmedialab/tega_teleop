# Jacqueline Kory Westlund
# July 2017
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

from PySide import QtGui # Basic GUI stuff.
from tega_teleop_ros import tega_teleop_ros
# Get fidget constants from TegaAction ROS msgs.
from r1d1_msgs.msg import TegaAction

class tega_fidget_ui(QtGui.QWidget):

    fidget_sets = {
            "no fidgets": TegaAction.FIDGETS_EMPTY,
            "speech fidgets": TegaAction.FIDGETS_SPEECH,
            "physical fidgets": TegaAction.FIDGETS_PHYSICAL
            }

    def __init__(self, ros_node):
        """ Make buttons to tell robot to fidget. """
        super(tega_fidget_ui, self).__init__()
        # Get reference to ros node so we can do callbacks to publish messages.
        self.ros_node = ros_node

        # Put buttons in a box.
        self.fidget_box = QtGui.QGroupBox(self)
        self.fidget_layout = QtGui.QGridLayout(self.fidget_box)
        self.fidget_box.setTitle("Fidgets")

        # Create fidget buttons and add to layout.
        self.label = QtGui.QLabel(self.fidget_box)
        self.label.setText("Choose fidget set: ")
        self.fidget_layout.addWidget(self.label, 0, 0)

        self.fidget_set_box = QtGui.QComboBox(self)
        fidget_set_list = list(self.fidget_sets.keys())
        self.fidget_set_box.addItems(fidget_set_list)
        self.fidget_set_box.activated['QString'].connect(self.on_fidget_set_selected)
        self.fidget_layout.addWidget(self.fidget_set_box, 1, 0, 1, 2)

    def on_fidget_set_selected(self, fidget_set):
        """ When the fidget set is changed in the combo box, send a message
        to the robot to tell it what set should now be in use.
        """
        self.ros_node.send_fidget_message(self.fidget_sets[fidget_set])
