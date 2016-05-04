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
from tega_teleop_ros import tega_teleop_ros
from geometry_msgs.msg import Vector3

class tega_lookat_ui(QtGui.QWidget):
    def __init__(self, ros_node):
        """ Make buttons to tell robot to look different directions """
        super(tega_lookat_ui, self).__init__()
        # get reference to ros node so we can do callbacks to
        # publish messages
        self.ros_node = ros_node
        
        # put buttons in a box
        lookat_box = QtGui.QGroupBox(self)
        lookat_layout = QtGui.QGridLayout(lookat_box)
        lookat_box.setTitle("Lookat")

        # create lookat buttons and add to layout
        # TODO calibrate lookat!
        # look stage left
        self.lbutton = QtGui.QPushButton("left", lookat_box) 
        self.lbutton.clicked.connect(lambda: 
                self.ros_node.send_lookat_message(Vector3(-50,0,0)))
        lookat_layout.addWidget(self.lbutton, 1, 0)
        # look center
        self.cbutton = QtGui.QPushButton("center", lookat_box) 
        self.cbutton.clicked.connect(lambda: 
                self.ros_node.send_lookat_message(Vector3(0,1,0)))
        lookat_layout.addWidget(self.cbutton, 1, 1)
        # look stage right
        self.rbutton = QtGui.QPushButton("right", lookat_box) 
        self.rbutton.clicked.connect(lambda: 
                self.ros_node.send_lookat_message(Vector3(50,0,0)))
        lookat_layout.addWidget(self.rbutton, 1, 2)
        # look up
        self.ubutton = QtGui.QPushButton("up", lookat_box) 
        self.ubutton.clicked.connect(lambda: 
                self.ros_node.send_lookat_message(Vector3(0,50,0)))
        lookat_layout.addWidget(self.ubutton, 0, 1)
        # look down
        self.dbutton = QtGui.QPushButton("down", lookat_box) 
        self.dbutton.clicked.connect(lambda: 
                self.ros_node.send_lookat_message(Vector3(0,-50,0)))
        lookat_layout.addWidget(self.dbutton, 2, 1)
        


