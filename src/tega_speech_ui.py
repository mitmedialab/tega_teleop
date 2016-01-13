from PySide import QtGui # basic GUI stuff
from tega_teleop_ros import tega_teleop_ros
import json
from functools import partial

class tega_speech_ui(QtGui.QWidget):

    # pause indicator
    paused = False

    # script list
    script_list = []

    # number of speech options per line in script
    options = 1

    # label for listing useful information for user
    label = None 


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
        # TODO dropdown list of available scripts to load, you pick one, it loads

        # add buttons to go forward and backward in the script
        # add buttons to pause and unpause the script (i.e., don't auto-advance)
        self.bbutton = QtGui.QPushButton("<< back", speech_box) 
        self.bbutton.clicked.connect(self.trigger_script_back)
        speech_layout.addWidget(self.bbutton, 0, 0)

        self.pbutton = QtGui.QPushButton("-- pause --", speech_box) 
        self.pbutton.clicked.connect(self.toggle_pause)
        speech_layout.addWidget(self.pbutton, 0, 1)

        self.fbutton = QtGui.QPushButton("forward >>", speech_box) 
        self.fbutton.clicked.connect(self.trigger_script_forward)
        speech_layout.addWidget(self.fbutton, 0, 2)

        self.sbutton = QtGui.QPushButton("[jump to start]", speech_box) 
        self.sbutton.clicked.connect(self.trigger_script_beginning)
        speech_layout.addWidget(self.sbutton, 0, 3)

        self.ebutton = QtGui.QPushButton("[jump to end]", speech_box) 
        self.ebutton.clicked.connect(self.trigger_script_end)
        speech_layout.addWidget(self.ebutton, 0, 4)

        self.label = QtGui.QLabel(speech_box)
        self.label.setText("Speech loaded.")
        speech_layout.addWidget(self.label, 1, 3, 2, 2)


        # read config file to get script name and number of speech options per line
        # NOTE move config parsing to main tega_teleop.py and pass script name and
        # number of options if we add anything not script/speech-related.
        try:
            with open("tega_teleop_config.json") as json_file:
                json_data = json.load(json_file)
            print(json_data)
            self.options = json_data["options"]
        except:
            pass
        
        # read in script
        script_file = open(json_data["script"])
        for line in script_file:
            self.script_list.append(line.rstrip().split("\t"))
        script_file.close()

        # start script line counter
        self.current_line = 0

        # set up the number of option buttons specified in config
        col = 0
        row = 1
        self.buttons = [None] * self.options 
        for i in range(0, self.options):
            text = self.script_list[self.current_line][i] if i < len(
                    self.script_list[self.current_line]) else "-"
            self.buttons[i] = QtGui.QPushButton(text.replace(".wav", ""), speech_box) 
            self.buttons[i].clicked.connect(partial(self.send_speech_command, 
                text, i))
            speech_layout.addWidget(self.buttons[i], row, 0, 1, 3)
            col += 2
            row += 1


    def toggle_pause(self):
        ''' pause or unpause auto-advance in script when speech buttons are pressed '''
        self.paused = not self.paused
        if (self.paused):
            self.pbutton.setStyleSheet('QPushButton {color: red;}')
            self.pbutton.setText("-- unpause --")
            self.label.setText("Paused.")
        else:
            self.pbutton.setStyleSheet('QPushButton {color: None;}')
            self.pbutton.setText("-- pause --")
            self.label.setText("Un-paused.")


    def trigger_script_beginning(self):
        ''' go to beginning of script '''
        self.current_line = 0
        self.update_speech_options()
        self.label.setText("At beginning of script.")


    def trigger_script_end(self):
        ''' go to end of script '''
        self.current_line = len(self.script_list) - 1 
        self.update_speech_options()
        self.label.setText("At end of script.")


    def trigger_script_back(self):
        ''' go to the previous line in the script '''
        # if the script isn't paused and we're not at the beginning, go back
        # and load the previous line of speech options
        if (self.paused):
            self.label.setText("Cannot go back! Script paused.")
            return

        if (self.current_line <= 0):
            self.label.setText("Cannot go back! At beginning.")
            return

        self.current_line -= 1
        self.update_speech_options()
         

    def trigger_script_forward(self):
        ''' go to the next line in the script '''
        # if the script isn't paused and we're not at the end, go forward
        # and load the next line of speech options
        if (self.paused):
            self.label.setText("Cannot go forward! Script paused.")
            return

        if (self.current_line >= len(self.script_list) - 1):
            self.label.setText("Cannot go forward! At end.")
            return

        self.current_line += 1
        self.update_speech_options()


    def update_speech_options(self):
        ''' update speech option buttons to go forward or back in script '''
        for i in range(0, self.options):
            text = self.script_list[self.current_line][i] if i < len(
                    self.script_list[self.current_line]) else "-"
            self.buttons[i].setText(text.replace(".wav", ""))
            self.buttons[i].clicked.disconnect()
            self.buttons[i].clicked.connect(partial(self.send_speech_command,
                text, i))
            self.label.setText("Next speech.")


    def send_speech_command(self, speech, option_num):
        ''' send speech command to robot and update speech options if necessary '''
        if (speech != "-"):
            # call ros send speech function
            self.ros_node.send_speech_message(speech)

        # if first option and not paused, autoadvance, call trigger script forward
        if (option_num == 0 and not self.paused):
            self.trigger_script_forward()
