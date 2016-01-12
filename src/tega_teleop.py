#!/usr/bin/env python
import rospy
import Tkinter as tk
from r1d1_msgs.msg import TegaAction
from r1d1_msgs.msg import TegaState
from sar_opal_msgs.msg import OpalCommand

class TegaTeleop(object):
    """ Tega teleoperation interface """
    # teleop interface elements
    roottk = tk.Tk()
    roottk.minsize(400,200)

    # grid locations
    # animations: 28 = 4 rows (rows 1,2,3,4)
    ANIM_START_COL = 0
    ANIM_START_ROW = 1
    ANIMS_PER_ROW = 7
    LOOKAT_START_COL = 0
    LOOKAT_START_ROW = 5

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

    # ROS node
    # start ROS node and list topics we publish
    rospy.init_node('tega_teleop', anonymous=True)
    tablet_pub = rospy.Publisher('opal_tablet_command', OpalCommand, queue_size = 10)
    tega_pub = rospy.Publisher('tega_action', TegaAction, queue_size = 10)
    r = rospy.Rate(10) # spin at 10 Hz
    #r.sleep() # sleep to wait for subscribers


    def __init__(self):
        """ Initialize teleop interface """
        # setup GUI teleop interface
        self.roottk.title=("Tega Teleop")
        self.roottk.grid()

        # setup animation trigger buttons
        self.animation_frame = tk.Frame(width=400,height=400)
        self.animation_frame.grid()
        self.setupAnimationButtons()
        sep = tk.Frame(height=2, bd=1, relief=tk.SUNKEN)
        sep.grid(sticky=tk.W+tk.E)
        
        # setup lookat buttons
        self.lookat_frame = tk.Frame(width=400,height=400)
        self.lookat_frame.grid()
        self.setupLookatButtons()
        sep = tk.Frame(height=2, bd=1, relief=tk.SUNKEN)
        sep.grid(sticky=tk.W+tk.E)

        # load script and speech buttons
        #self.setupSpeechButtons()


    def setupAnimationButtons(self):
        """ Make a button for each animation """
        self.anim_label = tk.Label(self.animation_frame, text="Animations:")
        self.anim_label.grid(column=0, row=0, sticky=tk.W, columnspan=2)

        col = self.ANIM_START_COL
        row = self.ANIM_START_ROW
        for anim in self.animations:
# TODO why is the same animation message being sent for each button?
            self.button = tk.Button(self.animation_frame, 
                    text=anim.lower().replace("\"", ""),
                    command=lambda: self.send_motion_message(anim))
            self.button.grid(column=col, row=row, sticky=tk.W)
            col += 1
            if(col >= self.ANIMS_PER_ROW):
                col = 0
                row += 1

    def setupLookatButtons(self):
        """ Make buttons to look different directions """
        # label the section
        self.lookat_label = tk.Label(self.lookat_frame, text="Look at:")
        self.lookat_label.grid(column=0, row=5, sticky=tk.W, columnspan=7)
        
        # look left
        # TODO calibrate lookat
        self.button = tk.Button(self.lookat_frame, 
                    text="left",
                    command=lambda: self.send_lookat_message((10,0,0)))
        self.button.grid(column=1, row=6, sticky=tk.W)

        # look center
        self.button = tk.Button(self.lookat_frame, 
                    text="center",
                    command=lambda: self.send_lookat_message((0,0,0)))
        self.button.grid(column=2, row=6, sticky=tk.W)
       
        # look right
        self.button = tk.Button(self.lookat_frame, 
                    text="right",
                    command=lambda: self.send_lookat_message((-10,0,0)))
        self.button.grid(column=3, row=6, sticky=tk.W)

        # look down
        self.button = tk.Button(self.lookat_frame, 
                    text="down",
                    command=lambda: self.send_lookat_message((0,-10,0)))
        self.button.grid(column=4, row=6, sticky=tk.W)
       
        # look up
        self.button = tk.Button(self.lookat_frame, 
                    text="up",
                    command=lambda: self.send_lookat_message((0,10,0)))
        self.button.grid(column=5, row=6, sticky=tk.W)

        
    #def setupSpeechButtons(self):
        """ Make buttons to trigger speech """
                

    def send_opal_message(self, command):
        """ Publish opal command message """
        print('sending opal command')
        msg = OpalCommand()
        msg.command = command
        self.tablet_pub.publish(msg)
        rospy.loginfo(msg)


# TODO combine these, one function with if-elses to pack msg?
    def send_motion_message(self, motion):
        """ Publish TegaAction do motion message """
        print('sending motion message')
        msg = TegaAction()
        msg.do_motion = True
        msg.motion = motion
        self.tega_pub.publish(msg)
        rospy.loginfo(msg)

    def send_lookat_message(self, lookat):
        """ Publish TegaAction lookat message """
        print('sending lookat message')
        msg = TegaAction()
        msg.do_look_at = True
        msg.look_at = lookat
        self.tega_pub.publish(msg)
        rospy.loginfo(msg)


    def start(self):
        """ Start the main GUI loop """
        # start main tk loop
        self.roottk.mainloop()
    

if __name__ == '__main__':
    # start teleop interface
    try:
        teleop = TegaTeleop()
        teleop.start()
    except rospy.ROSInterruptException: 
        print ('ROSnode shutdown')
        pass
