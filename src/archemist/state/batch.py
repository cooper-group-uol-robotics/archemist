from datetime import datetime
from archemist.state.material import Solid, Liquid
from enum import Enum

class BatchState(Enum):
    READY_FOR_PROCESSING = 0
    PROCESSING = 1
    PROCESING_DONE = 2
    TRANSIT = 3
    TRANSIT_DONE = 4

class Sample():
    def __init__(self, id: int, rack_indx: tuple):
        self._id = id
        self._rack_indx = rack_indx
        self._liquids = []
        self._solids = []
        self._location = None
        self._capped = False
        self._processed = False
        self._operationOps = []

    @property
    def id(self):
        return self._id

    @property
    def rack_indx(self):
        return self._rack_indx

    @property
    def liquids(self):
        return self._liquids

    @property
    def solids(self):
        return self._solids

    def addLiquid(self, liquid: Liquid):
        self._liquids.append(liquid)

    def addSolid(self, solid: Solid):
        self._solids.append(solid)

    @property
    def location(self):
        return self._location

    @location.setter
    def location(self, location):
        if isinstance(location, Location):
            self._location = location
        else:
            raise ValueError

    @property
    def capped(self):
        return self._capped

    @capped.setter
    def capped(self, value: bool):
        if isinstance(value, bool):
            self._capped = value
        else:
            raise ValueError

    @property
    def processed(self):
        return self._processed

    @processed.setter
    def processed(self, value: bool):
        if isinstance(value, bool):
            self._processed = value
        else:
            raise ValueError

    def addOpeationOp(self, opeation):
        self._operationOps.append((datetime.now(), opeation))

    @property
    def operationOps(self):
        return self._operationOps


    def getTotalMass(self):
        total_mass = 0
        for liquid in self._liquids:
            total_mass += liquid.mass
        for solid in self._solids:
            total_mass += solid.mass
        return total_mass


class Batch:

    def __init__(self, id: int, recipe, rows: int, cols: int):
        self.id = id
        self._location = None
        self._recipe = recipe
        self._state = BatchState.READY_FOR_PROCESSING
        self._num_sample = 0
        for i in range(0,rows):
            for j in range(0, cols):
                self._fresh_samples.append(Sample(id,(i,j)))
                _num_sample += 1
        self._processed_samples = []
        self._station_history = []

    @property
    def id(self):
        return self._id

    @property
    def recipe(self):
        return self._recipe

    @property
    def location(self):
        return self._location

    @location.setter
    def location(self, location):
        if isinstance(location, Location):
            self._location = location
        else:
            raise ValueError

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, state: BatchState):
        if isinstance(state, BatchState):
            self._state = state
        else:
            raise ValueError


    def addStationStamp(self, station_name: str):
        self._station_history.append((datetime.now(), station_name))

    def getLastStationStamp(self):
        return self._station_history[-1]

    @property
    def station_history(self):
        return self._station_history

    def getCurrentFlowNode(self):
        return self._recipe.currentNode

    @property
    def samples(self):
        return self._samples

    def getFreshSample(self):
        self._state = BatchState.PROCESING
        return self._fresh_samples.pop()

    def putProcessedSample(self, processed_sample:Sample):
       self._processed_samples.append(processed_sample)
       if (len(self._processed_samples) ==  self._num_sample):
           self._state = BatchState.PROCESING_DONE

    def resetBatchForProcessing(self):
        if self._state == BatchState.PROCESING_DONE:
            for i in self._processed_samples[:]:
                self._fresh_samples.append(i)
                self._processed_samples.remove(i)
            self._state = BatchState.READY_FOR_PROCESSING
