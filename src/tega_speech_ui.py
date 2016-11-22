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
import json
import glob
from functools import partial
import time

class tega_speech_ui(QtGui.QWidget):


    def __init__(self, ros_node, flags):
        """ Make controls to trigger speech playback """
        super(tega_speech_ui, self).__init__()
        # get reference to ros node so we can do callbacks to publish
        # messages
        self.ros_node = ros_node
    
        # pause indicator
        self.paused = False

        # script list
        self.script_list = []

        # number of speech options per line in script
        self.options = 1

        # label for listing useful information for user
        self.label = None 

        # get reference to shared flags, such as whether child is attending
        # or not 
        self.flags = flags
        
        # put buttons in a box
        self.speech_box = QtGui.QGroupBox(self)
        self.speech_layout = QtGui.QGridLayout(self.speech_box)
        self.speech_box.setTitle("Speech")

        # get speech and add speech playback buttons to layout:
       
        # add buttons to go forward and backward in the script
        # add buttons to pause and unpause the script (i.e., don't
        # auto-advance)
        self.bbutton = QtGui.QPushButton("<< back", self.speech_box) 
        self.bbutton.clicked.connect(self.trigger_script_back)
        self.speech_layout.addWidget(self.bbutton, 1, 0)

        self.pbutton = QtGui.QPushButton("-- pause --", self.speech_box) 
        self.pbutton.clicked.connect(self.toggle_pause)
        self.speech_layout.addWidget(self.pbutton, 1, 1)

        self.fbutton = QtGui.QPushButton("forward >>", self.speech_box) 
        self.fbutton.clicked.connect(self.trigger_script_forward)
        self.speech_layout.addWidget(self.fbutton, 1, 2)

        self.sbutton = QtGui.QPushButton("[jump to start]", self.speech_box) 
        self.sbutton.clicked.connect(self.trigger_script_beginning)
        self.sbutton.setStyleSheet('QPushButton {color: gray;}')
        self.speech_layout.addWidget(self.sbutton, 1, 3)

        self.ebutton = QtGui.QPushButton("[jump to end]", self.speech_box) 
        self.ebutton.clicked.connect(self.trigger_script_end)
        self.ebutton.setStyleSheet('QPushButton {color: gray;}')
        self.speech_layout.addWidget(self.ebutton, 1, 4)

        self.label = QtGui.QLabel(self.speech_box)
        self.label.setText("---")
        self.speech_layout.addWidget(self.label, 2, 0, 1, 3)

        json_data=[]

        # read config file to get script name and number of speech options 
        # per line
        # NOTE move config parsing to main tega_teleop.py and pass script name
        # and number of options if we add anything not script/speech-related.
        try:
            with open("tega_teleop_config.json") as json_file:
                json_data = json.load(json_file)
            print ("Config file says: ")
            print (json_data)
            if ("options" in json_data):
                self.options = json_data["options"]
            else:
                self.options = 1
                print ("Could not read number of options! Set to default of 1.")
        except:
            print ("Could not read your json config file! Is it valid json?")
            pass

        # TODO add the file paths to folders of scripts into config file!
        # make a dropdown list of available scripts to load 
        # user picks one, it loads
        script_box_label = QtGui.QLabel(self.speech_box)
        script_box_label.setText("Pick a script to load: ")
        self.speech_layout.addWidget(script_box_label, 0, 0)
        self.script_list_box = QtGui.QComboBox(self)
        script_file_list = glob.glob('../scripts/*.txt')
        self.script_list_box.addItems(script_file_list)
        self.script_list_box.activated['QString'].connect(self.load_script)
        self.speech_layout.addWidget(self.script_list_box, 0, 1, 1, 2)

        # make a dropdown list of available static scripts to load 
        # user picks one, it loads
        self.static_script_list_box = QtGui.QComboBox(self)
        static_script_file_list = glob.glob('../static_scripts/*.txt')
        self.static_script_list_box.addItems(static_script_file_list)
        self.static_script_list_box.activated['QString'].connect(
               self.load_static_script)
        self.speech_layout.addWidget(self.static_script_list_box, 0, 3, 1, 2)

        # read in script if we can
        if ("script" in json_data):
            self.load_script(json_data["script"])

        else:
            print("Could not load script! Is your config file correct?")
            self.label.setText("Could not load script!")

        # set up buttons for speech options that are always available
        # using the "unchanging script" file
        # read in that script
        if ("static_script" in json_data):
            self.load_static_script(json_data["static_script"])
        else:
            print("Should there be a static script in your config file?")


    def load_script(self, script_filename):
        ''' load a script file '''
        print("loading script...")
        try:
            # reset list holding the script lines
            self.script_list = []
            # read in script
            script_file = open(script_filename)
            for line in script_file:
                self.script_list.append(line.rstrip().split("\t"))
            script_file.close()

            # start script line counter
            self.current_line = 0

            # set up the number of option buttons specified in config:
            # where we are putting these buttons in the grid
            col = 0
            row = 3

            # remove old buttons if there were any
            try:
                for b in self.buttons:
                    self.speech_layout.removeWidget(b)
                    b.deleteLater()
                    b = None
            except AttributeError:
                print "No script buttons yet... let's set some up."
                
            # make new array of buttons for the number of speech options
            # that this new script to load has
            self.buttons = [None] * self.options 

            # each script line follows the pattern:
            # filename1 label1 filename2 label2 ... etc.
            for i in range(0, self.options):
                # set button text to the button label
                self.buttons[i] = QtGui.QPushButton(self.script_list[
                self.current_line][i*2+1] if i < len(self.script_list[
                    self.current_line])/2 else "-", self.speech_box) 
                # when clicked, call send_speech_command with the argument
                # that is the filename for the audio to play
                # note: the speech option may be a comma separated list
                # where one item is the filename and one is an animation to
                # play back before or after the file
                self.buttons[i].clicked.connect(partial(self.send_speech_command, 
                    self.script_list[self.current_line][i*2] if i < len(
                        self.script_list[self.current_line])/2 else "-", i))
                # add button to layout, each button takes up three columns
                self.speech_layout.addWidget(self.buttons[i], row, 0, 1, 3)
                col += 2
                row += 1
            # make the first option green since clicking it will auto-advance
            # the script and update the buttons
            self.buttons[0].setStyleSheet('QPushButton {color: green;}')
            self.label.setText("Script loaded!")
        except:
            print ("Could not read script file! Is filename in config correct?")
            self.label.setText("Could not read script file!")
        

    def load_static_script(self, script_filename):
        ''' load a script file '''
         # remove old buttons if there were any
        try:
            for b in self.static_buttons:
                self.speech_layout.removeWidget(b)
                b.deleteLater()
                b = None
        except AttributeError:
            print "No static script buttons yet... let's set some up."

        # make new list of buttons for the static script options
        self.static_buttons = []

        try:
            row = 3 
            static_script = open(script_filename)

            for line in static_script:
                parts = line.rstrip().split("\t")
                # set button text to the button label if a label was provided
                button = QtGui.QPushButton(parts[1] if len(
                    parts) > 1 else parts[0], self.speech_box)
                # send filename of audio to play when button is clicked
                button.clicked.connect(partial(self.send_speech_command,
                    parts[0].split(",")[0], -1))
                # make button text purple so they are distinct
                button.setStyleSheet('QPushButton {color: purple;}')
                self.speech_layout.addWidget(button, row, 3, 1, 2) 
                self.static_buttons.append(button)
                row += 1
        except:
            print ("Could not read static script file! Is filename correct?")


    def toggle_pause(self):
        ''' pause or unpause auto-advance script when speech buttons are pressed '''
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
            # set button text to the button label
            # if there are more buttons than speech options for this line in
            # the script, then set the text to "-"
            self.buttons[i].setText(self.script_list[self.current_line][
                i*2+1] if i < len(self.script_list[self.current_line])/2 else "-")
            # disconnect previous callback function
            try:
                self.buttons[i].clicked.disconnect()
            except:
                print("oops, tried to disconnect a button that wasn't connected")
            # when clicked, call send_speech_command with the argument
            # that is the filename for the audio to play
            # if there are more buttons than speech options for this line in
            # the script, then send a "-" when clicked instead
            self.buttons[i].clicked.connect(partial(self.send_speech_command, 
                self.script_list[self.current_line][i*2] if i < len(
                    self.script_list[self.current_line])/2 else "-", i))
            self.label.setText("Next speech.")


    def send_speech_command(self, speech, option_num):
        ''' send speech command to robot and update speech options if necessary '''
        if (speech != "-"):
            # split command on commas, find out if there's just speech or
            # animations listed
            speech_parts = speech.split(",")

            # send a command for each part found
            for sp in speech_parts:
                # wait until tega is not speaking or moving, then send the
                # next command
                # TODO text this with new robot rig
                while (self.flags.tega_is_playing_sound
                       or self.flags.tega_is_doing_motion):
                    time.sleep(0.5)

                # if this part is an animation (all caps), send a motion command
                if (sp.isupper()):
                    self.ros_node.send_motion_message(sp)
                    self.label.setText("Sending animation.")
                # otherwise, it's a speech filename, so call ros send speech
                else: 
                    # call ros send speech function
                    self.ros_node.send_speech_message(sp)
                    self.label.setText("Sending speech command.")
                time.sleep(0.5)

        speech = "-"
        # if first option and not paused, autoadvance, call trigger script forward
        if (option_num == 0 and not self.paused):
            self.trigger_script_forward()

        # TODO move project-specific stuff like the redirects and child attention
        # label to a forked version of the project OR add a project-specific 
        # python file to load where you add any project-specific buttons to the
        # interface -- something to make this cleaner. Anyway:
        #
        # if we are using redirects, and if the child is not attending,
        # highlight the redirects so the teleoperator will know to click one 
        #
        # NOTE there may be a better way of doing this - the coloring is 
        # dependent on the value of the flag when we send speech, and it could
        # be that the child is not attending for a bit but comes back before
        # it's time to send more speech, so we might want to play a redirect
        # since they were distracted, but the color won't change for that...
        # the point is, we could track how much the child has been attending
        # since the last time we changed the button colors or sent speech, and 
        # use that to determine whether we should suggest playing another 
        # redirect or not.
        print self.flags.child_is_attending
        if self.flags.child_is_attending:
            for sb in self.static_buttons:
                sb.setStyleSheet('QPushButton {color: purple;}')
        else:
            for sb in self.static_buttons:
                sb.setStyleSheet('QPushButton {color: red;}')

       
