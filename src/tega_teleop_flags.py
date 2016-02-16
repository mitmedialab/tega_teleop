
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
