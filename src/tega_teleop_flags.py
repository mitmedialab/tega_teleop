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

class tega_teleop_flags:

    # is the child attending or not?
    _child_is_attending = True
    @property
    def child_is_attending(self):
        return self._child_is_attending
    @child_is_attending.setter
    def child_is_attending(self,val):
        self._child_is_attending = val

    # is tega currently playing audio? 
    # we get this info from the tega state rosmsgs
    _tega_is_playing_sound = False
    @property
    def tega_is_playing_sound(self):
        return self._tega_is_playing_sound
    @tega_is_playing_sound.setter
    def tega_is_playing_sound(self,val):
        self._tega_is_playing_sound = val

    # is tega currently doing a motion? 
    # we get this info from the tega state rosmsgs
    _tega_is_doing_motion = False
    @property
    def tega_is_doing_motion(self):
        return self._tega_is_doing_motion
    @tega_is_playing_sound.setter
    def tega_is_doing_motion(self,val):
        self._tega_is_doing_motion = val
