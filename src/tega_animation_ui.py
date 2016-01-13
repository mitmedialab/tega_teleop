from PySide import QtGui # basic GUI stuff
from r1d1_msgs.msg import TegaAction # ROS msgs

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


    def __init__(self):
        super(tega_animation_ui, self).__init__()
        
        """ Make a button for each animation """
        # put buttons in a box
        anim_box = QtGui.QGroupBox(self)
        #central_layout.addWidget(anim_box)
        anim_layout = QtGui.QGridLayout(anim_box)
        anim_box.setTitle("Animations")
        #label = QtGui.QLabel(self.central_widget)
        #label.setFrameStyle(QtGui.QFrame.Panel)
        #label.setText("Animations")
        #anim_layout.addWidget(label, 0, 0, 1, 4)

        # create animation buttons and add to layout
        col = 0
        row = 1
        for anim in self.animations:
            self.button = QtGui.QPushButton(anim.lower().replace("\"", ""), 
                    anim_box)
            self.button.clicked.connect(lambda: self.send_motion_message(anim))
            anim_layout.addWidget(self.button, row, col)
            col += 1
            if(col >= 4): # four animation buttons per row
                col = 0
                row += 1

