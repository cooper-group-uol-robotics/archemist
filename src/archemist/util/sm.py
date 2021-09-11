from enum import Enum

class States(Enum):
    TRANSPORTING_RACK = 1
    PLACING_RACK = 2
    PRE_LOAD_VIAL = 3
    LOAD_VIAL = 4
    PROCESS_VIAL = 5
    POST_LOAD_VIAL = 6
    RETURN_VIAL = 7
    PICKING_RACK = 8
    RESET = 9

class ProcessSM():
    def __init__(self):
        self._state = States.TRANSPORTING_RACK
    
    def advanceState(self):
        if (self._state == States.TRANSPORTING_RACK):
            self._state = States.PLACING_RACK
        elif (self._state == States.PLACING_RACK):
            self._state = States.PRE_LOAD_VIAL
        elif (self._state == States.PRE_LOAD_VIAL):
            self._state = States.LOAD_VIAL
        elif (self._state == States.LOAD_VIAL):
            self._state = States.PROCESS_VIAL
        elif (self._state == States.PROCESS_VIAL):
            self._state = States.POST_LOAD_VIAL
        elif (self._state == States.POST_LOAD_VIAL):
            self._state = States.RETURN_VIAL
        elif (self._state == States.RETURN_VIAL):
            self._state = States.PICKING_RACK
        elif (self._state == States.PICKING_RACK):
            self._state = States.RESET

    def reset(self):
        self._state = States.TRANSPORTING_RACK

    @property
    def state(self):
        return self._state