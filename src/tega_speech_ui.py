from PySide import QtGui # basic GUI stuff
from tega_teleop_ros import tega_teleop_ros

class tega_speech_ui(QtGui.QWidget):
    def __init__(self, ros_node):
        """ Make controls to trigger speech playback """
        super(tega_speech_ui, self).__init__()
        # get reference to ros node so we can do callbacks to publish messages
        self.ros_node = ros_node
        
        # put buttons in a box
        speech_box = QtGui.QGroupBox(self)
        speech_layout = QtGui.QGridLayout(speech_box)
        speech_box.setTitle("Speech")

        # get speech and add speech playback buttons to layout

