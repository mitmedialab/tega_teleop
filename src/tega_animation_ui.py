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
            TegaAction.MOTION_EXCITED,
            TegaAction.MOTION_INTERESTED,
            TegaAction.MOTION_IDLESTILL,
            TegaAction.MOTION_NOD, 
            TegaAction.MOTION_PERKUP, 
            TegaAction.MOTION_DANCE, 
            TegaAction.MOTION_HAPPY_DANCE, 
            TegaAction.MOTION_ROCKING,
            TegaAction.MOTION_SWAY,
            TegaAction.MOTION_HAPPY_UP, 
            TegaAction.MOTION_HAPPY_WIGGLE, 
            TegaAction.MOTION_NO,
            TegaAction.MOTION_SAD,
            TegaAction.MOTION_YAWN,
            TegaAction.MOTION_THINKING,
            TegaAction.MOTION_PUZZLED,
            TegaAction.MOTION_FRUSTRATED,
            TegaAction.MOTION_POSE_BREATHING,
            TegaAction.MOTION_POSE_FORWARD,
            TegaAction.MOTION_POSE_SLEEPING,
            TegaAction.MOTION_POSE1,
            TegaAction.MOTION_POSE2,
            TegaAction.MOTION_SHIFT_WEIGHT1,
            TegaAction.MOTION_SHIFT_WEIGHT2,
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
            anim_layout.addWidget(button, row, col)
            col += 1
            if(col >= 4): # four animation buttons per row
                col = 0
                row += 1
