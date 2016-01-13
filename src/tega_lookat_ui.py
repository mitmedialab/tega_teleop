from PySide import QtGui # basic GUI stuff
from tega_teleop_ros import tega_teleop_ros

class tega_lookat_ui(QtGui.QWidget):
    def __init__(self, ros_node):
        """ Make buttons to tell robot to look different directions """
        super(tega_lookat_ui, self).__init__()
        # get reference to ros node so we can do callbacks to publish messages
        self.ros_node = ros_node
        
        # put buttons in a box
        lookat_box = QtGui.QGroupBox(self)
        lookat_layout = QtGui.QGridLayout(lookat_box)
        lookat_box.setTitle("Lookat")

        # create lookat buttons and add to layout
        # TODO calibrate lookat!
        # look left
        self.lbutton = QtGui.QPushButton("left", lookat_box) 
        self.lbutton.clicked.connect(lambda: self.ros_node.send_lookat_message((10,0,0)))
        lookat_layout.addWidget(self.lbutton, 1, 0)
        # look center
        self.cbutton = QtGui.QPushButton("center", lookat_box) 
        self.cbutton.clicked.connect(lambda: self.ros_node.send_lookat_message((0,0,0)))
        lookat_layout.addWidget(self.cbutton, 1, 1)
        # look right
        self.rbutton = QtGui.QPushButton("right", lookat_box) 
        self.rbutton.clicked.connect(lambda: self.ros_node.send_lookat_message((10,0,0)))
        lookat_layout.addWidget(self.rbutton, 1, 2)
        # look up
        self.ubutton = QtGui.QPushButton("up", lookat_box) 
        self.ubutton.clicked.connect(lambda: self.ros_node.send_lookat_message((0,10,0)))
        lookat_layout.addWidget(self.ubutton, 0, 1)
        # look down
        self.dbutton = QtGui.QPushButton("down", lookat_box) 
        self.dbutton.clicked.connect(lambda: self.ros_node.send_lookat_message((0,-10,0)))
        lookat_layout.addWidget(self.dbutton, 2, 1)
        


