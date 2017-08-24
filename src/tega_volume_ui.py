# Jacqueline Kory Westlund
# August 2017
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

class tega_volume_ui(QtGui.QWidget):

    def __init__(self, ros_node):
        """ Make buttons to tell robot to play audio louder or quieter. """
        super(tega_volume_ui, self).__init__()
        # Get reference to ros node so we can do callbacks to publish messages.
        self.ros_node = ros_node

        # Put buttons in a box.
        self.volume_box = QtGui.QGroupBox(self)
        self.volume_layout = QtGui.QGridLayout(self.volume_box)
        self.volume_box.setTitle("Volume")

        # Create volume buttons and add to layout.
        self.label = QtGui.QLabel(self.volume_box)
        self.label.setText("Set volume (0=none, 1=max): ")
        self.volume_layout.addWidget(self.label, 0, 0)
        self.volume_spin_box = QtGui.QDoubleSpinBox(self)
        self.volume_spin_box.setRange(0.0, 1.0)
        self.volume_spin_box.setValue(0.5)
        self.volume_spin_box.setSingleStep(0.05)
        self.volume_spin_box.valueChanged[float].connect(self.on_volume_changed)
        self.volume_layout.addWidget(self.volume_spin_box, 1, 0, 1, 1)

    def on_volume_changed(self, volume):
        """ When the volume is changed, send a message to the robot to tell it
        what the volume should be.
        """
        self.ros_node.send_volume_message(volume)
