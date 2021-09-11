from datetime import datetime
from archemist.state.material import Solid, Liquid
from archemist.util import Location
from archemist.util.sm import ProcessSM

class Sample():
    def __init__(self, id: int, rack_indx: tuple, location: Location, recipe):
        self._id = id
        self._rack_indx = rack_indx
        self._liquids = []
        self._solids = []
        self._location = location
        self._recipe = recipe
        self._capped = False
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

    def addOpeationOp(self, opeation):
        self._operationOps.append(opeation)
    
    def getCurrentOp(self):
        self._operationOps[-1]

    @property
    def operationOps(self):
        return self._operationOps

    @property
    def recipe(self):
        return self._recipe

    def getCurrentFlowNode(self):
        return self._recipe.stationFlow.currentNode


    def getTotalMass(self):
        total_mass = 0
        for liquid in self._liquids:
            total_mass += liquid.mass
        for solid in self._solids:
            total_mass += solid.mass
        return total_mass


class Batch:

    def __init__(self, id: int, recipe, rows: int, cols: int, location: Location):
        self._id = id
        self._location = location
        self._recipe = recipe
        self._recipe.advanceNode(True) # to move out of start state
        self._samples = list()
        for i in range(0,rows):
            for j in range(0, cols):
                self._samples.append(Sample(id,(i,j),location,recipe))
        
        self._assigned = False
        self._all_processed = False
        
        #self._current_sample = self._samples[0]
        self._processed_samples = []
        
        self._station_history = []
        self._operationOps = []

        self.processSM = ProcessSM()

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
            for sample in self._samples:
                sample.location = location
        else:
            raise ValueError

    @property
    def assigned(self):
        return self._assigned

    @assigned.setter
    def assigned(self, value):
        if isinstance(value, bool):
            self._assigned = value
        else:
            raise ValueError

    def addStationStamp(self, station_name: str):
        self._station_history.append((datetime.now(), station_name))

    def getLastStationStamp(self):
        return self._station_history[-1]

    @property
    def station_history(self):
        return self._station_history

    def getCurrentStation(self):
        return self._recipe.getCurrentNode()

    def getUpcomingStation(self):
        return self._recipe.stationFlow.getUpcomingNode()

    def advanceRecipeState(self, success: bool):
        #single batch mode here
        self._recipe.advanceNode(success)
        if self._recipe.hasEnded():
            self._processed_samples.append(self._samples.pop(0))
            if not self._samples:
                # done will all samples
                self._all_processed = True
            else:
                self._current_sample = self._samples[0]
                self._recipe.reset()

    def batchComplete(self):
        return self._all_processed

    def advanceProcessState(self):
        self.processSM.advanceState()

    def resetProcessState(self):
        self.processSM.reset()


    def addOpeationOp(self, opeation):
        self._operationOps.append(opeation)
    
    def getCurrentOp(self):
        self._operationOps[-1]
    
    def getCurrentSample(self):
        return self._samples[0]
