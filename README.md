# tega\_teleop

A python rosnode for teleoperating the Tega robot and an opal tablet. Creates a
Qt GUI with buttons for triggering speech, lookats, and animations on the
robot, as well as commands to send to an opal tablet.

## Configure and Run

`python tega_teleop.py [-h] [-e]`

optional arguments:
    - `-h`, `--help`: show this help message and exit
    - `-e`, `--use-entrainer`: Send audio to the audio entrainer on the way to
      the robot.

On startup, this python node will try to connect to roscore. If roscore is not
running, the program will exit.

If this node is running on a network where DNS does not resolve local
hostnames, you will need to export the environment variable $ROS\_IP and
$ROS\_HOSTNAME to be the public IP address of this node. For example, if the
machine this node is running on has the IP address "192.168.1.20", you would
run the commands `export ROS\_IP=192.168.1.20` and `export
ROS\_HOSTNAME=192.168.1.20` in your shell prior to starting this node. If this
IP is static, you may want to put these commands in your bashrc file (or other
shell rc file)  so you don't have to remember to run them every time.

### Config

The program will use values from the config file to load scripts and send audio:

    - script: the interaction script to use.
    - options: how many speech buttons are shown for each line of the script.
    - static\_script: a list of "always there" speech buttons.
    - audio\_base\_dir: a directory containing audio files (only used if the
      audio entrainer will be used)

More detail about all these options is provided below.

Essentially, the program will try to read in the interaction script file listed
in the "tega\_teleop\_config.txt" file. There is an example interaction script
file located in src/. This script file should list, in order, the filenames of
all audio files that the robot will be commanded to play back during the
interaction. The second column should list the words that shown on the speech
buttons in the GUI (for example, the filename may be "robot\_line\_01.wav" and
the words to show may be "Hi, I'm Tega"). This allows the program to be used
for different interactions and different sets of audio files without having to
change the python code for creating buttons.


### More on scripts

#### Basic scripts

The most basic script would be a single list of filenames with a single list of
button labels. The robot is commanded to say one thing, then the next thing,
and so on--always the same things, and always in the same order.

In a slightly more complicated script, the teleoperator may need to decide
which of several different things the robot should say next. For example, the
teleoperator might have to choose to have the robot say "Awesome!", "Hmm...",
or "Aw, are you sure?" in response to something a child says during the
interaction. That is three different options.

This interface can deal with that level of complication. All you need to do is
list the maximum number of different options the teleoperator will have (for
example, 3) in the config file: set "options" to "3". Then, on each line of
your script file, list the filenames of the teleoperator's speech options and
their button labels in tab-delimited format: \[filename1 label1 filename2
label2 etc.\]. When the script is loaded, whenever there are multiple options,
these will be shown on buttons simultaneously so the teleoperator can choose
which to trigger. You probably don't want to list more than 2-4 options for any
line of speech, since the teleoperator *does* have to keep track of when to use
each one. Increased cognitive load and all.

#### Animations in scripts

You can also list motions/animations to play back along with an audio file:
\[filename1,MOTION label1 filename2,MOTION etc.\]. When the button is clicked,
the interface will send both a "play audio" message with the audio filename and
a "do motion" message with the animation to play. If you list the filename
first, the play audio message will be sent first; if you list the animation
name first, the do motion message will be sent first. It is, however, up to the
robot code to determine if these are cued or whether they play simultaneously,
so make sure to test it for your use case.

#### PARTICIPANT\_TURN lines

You can include the phrase "PARTICIPANT_TURN" in the list of things to do. So a
line could include filenames of audio to play, animations, *and* indicators
that it is now the participant's turn to speak. This will send a message
containing interaction state information so that other modules can know that it
is the participant's turn.

A line might look like \[filename1,MOTION,PARTICIPANT_TURN label1
filename2,MOTION etc.\]

The assumption here is that the script writer knows when the participant is
asked for input and that the script lines have thus been written appropriately.

#### Static scripts

There is the option of including a set of "static script" buttons to trigger
speech. This is useful if, for example, the robot could always have the option
of saying "Mmhm" and "Awesome!" regardless of the rest of the script, and if
you don't want to include these phrases as options on every single line of your
main script file. Following the example\_static\_script.txt file, make a list
with one column of audio filenames and one tab-delimited column of button
labels.

You probably shouldn't list more than 3-5 of these "always there" speech
buttons, since otherwise the interface may start to look clunky with too many
buttons.

#### Dynamically loading scripts

One more thing on scripts. The interface will load a list of scripts located in
tega\_teleop\scripts (sibling to \src). There is a dropdown box where you can
select which script to load. This way, you can load new scripts or switch
scripts without needing to change the config file! Same deal for static
scripts, only they are loaded from the tega\_teleop\static\_scripts directory
(sibling to \src and \scripts).

### Audio entrainer

The [audio entrainer](https://github.com/mitmedialab/rr_audio_entrainer)
expects full filepaths when you send it audio (i.e., it expects you to give it
a wav file it can open). So to accommodate this, you can either specify full
filepaths in your script, or, in the config file, specify a directory on your
file system where you have put all the wav files that will be used. These then
get streamed to the robot. But you should only do one or the other!

If you put full filepaths in your script, you should leave the `audio_base_dir`
option in the config file blank (i.e., put an empty string there or delete it
entirely from the file),

### Opal tablet communication

Commands to the [opal tablet](https://github.com/mitmedialab/SAR-opal-base) are
sent over a rosbridge\_server websocket connection. For communication with the
tablet to occur, you need to have the rosbridge\_server running, using the
following command:

`roslaunch rosbridge_server rosbridge_websocket.launch`

You will also need to ensure that the opal tablet's config file lists the IP
address or hostname of the machine running roscore. The [opal
tablet](https://github.com/mitmedialab/SAR-opal-base) documentation explains
how to update the config file (it's simple; you change a line in a text file
and copy it to the tablet).

## ROS messages

### SAR Opal messages

The program publishes
"/[sar\_opal\_msgs](https://github.com/mitmedialab/sar_opal_msgs
"/sar_opal_msgs")/OpalCommand" to the ROS topic "/opal\_tablet\_command". See
[/sar\_opal\_msgs](https://github.com/mitmedialab/sar_opal_msgs
"/sar_opal_msgs") for more info.

### R1D1 messages

The program publishes "/[r1d1\_msgs](https://github.com/mitmedialab/r1d1_msgs
"/r1d1_msgs")/TegaAction" to the ROS topic "/tega".

The program subscribes to
"/[r1d1\_msgs](https://github.com/mitmedialab/r1d1_msgs
"/r1d1_msgs")/TegaState" on the ROS topic "/tega\_state".

See [/r1d1\_msgs](https://github.com/mitmedialab/r1d1_msgs "/r1d1_msgs") for
more info.

### Affdex attention messages

The program subscribes to Boolean (True/False) messages on the ROS topic
"/child\_attention". These messages indicate whether a child is attending to
the robot/tablet setup or not.

### Relational robot messages

The node publishes
"/[rr_msgs](https://github.com/mitmedialab/rr_msgs)/EntrainAudio" messages on
the ROS topic "/rr/entrain_audio".

The node also publishes
"/[rr_msgs](https://github.com/mitmedialab/rr_msgs)/InteractionState" messages
on the ROS topic "/rr/state".


## Version Notes

This program was developed and tested with:

- Python 2.7.6
- ROS Indigo
- [sar\_opal\_msgs](https://github.com/mitmedialab/sar_opal_msgs
  "/sar_opal_msgs") 4.0.0
- [r1d1\_msgs](https://github.com/mitmedialab/r1d1_msgs) 4.0.0
- [rr\_msgs](https://github.com/mitmedialab/rr_msgs) 1.0.0
- Ubuntu 14.04 LTS (32-bit)

The Cyber4 study was run using tega\_teleop v1.0.1.

## Bugs and issues

Please report all bugs and issues on the [tega\_teleop github issues
page](https://github.com/mitmedialab/tega_teleop/issues).

## TODO

- Set ROS\_IP from within the python script so the user doesn't have to
  remember to do it
- Adjust grid layout row height to make GUI look nicer
- Add the file paths to folders of scripts into config file
- Could we put a list of nodes to subscribe to in the config file?
- Move project-specific stuff to a separate script; make it so project-specific
  stuff can easily be swapped out for different projects

