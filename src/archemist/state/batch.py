from datetime import datetime
from archemist.state.material import Solid, Liquid
from archemist.state.station import Location, StationOpDescriptor, StationOutputDescriptor
from archemist.state.recipe import Recipe

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
    def location(self, location: Location):
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

    def addOpeationOp(self, opeation: StationOpDescriptor):
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

    def __init__(self, id: int, recipe: Recipe, rows: int, cols: int):
        self.id = id
        self._location = None
        self._recipe = recipe
        id = 1
        for i in range(0,rows):
            for j in range(0, cols):
                self._samples.append(Sample(id,(i,j)))
                id += 1
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
    def location(self, location: Location):
        if isinstance(location, Location):
            self._location = location
        else:
            raise ValueError

    def addStationStamp(self, station_name: str):
        self._station_history.append((datetime.now(), station_name))

    @property
    def station_history(self):
        return self._station_history

    @property
    def samples(self):
        return self._samples

    def getFreshSample(self):
        for sample in self._samples:
            if (not sample.processed):
                return sample

    def putProcessedSample(self, processed_sample:Sample):
        for sample in self._samples:
            if (processed_sample.id == sample.id):
                sample.processed = True

    def isBatchProcessed(self):
        for sample in self._samples:
            if (not sample.processed):
                return False
        return True

    def resetBatchForProcessing(self):
        for sample in self._samples:
            sample.processed = False

    
