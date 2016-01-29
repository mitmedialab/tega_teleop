
class tega_teleop_flags:

    # is the child attending or not?
    _child_is_attending = False
    @property
    def child_is_attending(self):
        return _child_is_attending
    @child_is_attending.setter
    def child_is_attending(self,val):
        self._child_is_attending = val
