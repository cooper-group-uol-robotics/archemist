from archemist.state.station import Station, Location, StationOpDescriptor, StationOutputDescriptor
from archemist.state.material import Solid
from archemist.exceptions.exception import UsingConsumedCatridgeError, QuantosRackLoadedError

class QuantosCatridge():
    def __init__(self, solid: Solid, remaining_dosages: int, blocked: bool, hotel_id: int):
        self._solid = solid
        self._remaining_dosages = remaining_dosages
        self._blocked = False
        self._consumed = False
        self._loaded_to_quantos = False
        self._hotel_id = hotel_id

        @property
        def solid(self):
            return self._solid

        @property
        def hotel_id(self):
            return self._hotel_id

        @property
        def consumed(self):
            return self._consumed

        @property
        def blocked(self):
            return self._blocked

        @blocked.setter
        def blocked(self, value):
            if isinstance(value, bool):
                self._blocked = value
            else:
                raise ValueError

        @property
        def loaded_to_quantos(self):
            return self._loaded_to_quantos

        @loaded_to_quantos.setter
        def loaded_to_quantos(self, value):
            if isinstance(value, bool):
                self._loaded_to_quantos = value
            else:
                raise ValueError

        @property
        def remaining_dosages(self):
            return self._remaining_dosages

        def dispense(self, dispensed_solid: Solid):
            if (not self._consumed):
                self._solid.mass = self._solid.mass - dispensed_solid.mass
                self._remaining_dosages -= 1
                if (_remaining_dosages <= 0):
                    self._consumed = True
            else:
                raise UsingConsumedCatridgeError(self._hotel_id)





class QuantosSolidDispenserQS2(Station):
    def __init__(self, id: int, loc: Location, catridges: list):
        super().__init__(id, loc)
        self._carouselPos = -1
        self._catridges = catridges
        self._current_catridge = None
        self._doors_open = True

    @property
    def carouselPos(self):
        return self._carouselPos

    @carouselPos.setter
    def carouselPos(self, value: int):
        if (value <= 20 and value >= 1):
            self._carouselPos = value
        else:
            raise ValueError

    @property
    def current_catridge(self):
        return self._current_catridge

    @property
    def doors_open(self):
        return self._doors_open

    @doors_open.setter
    def doors_open(self, value):
        if isinstance(value, bool):
            self._doors_open = value
        else:
            raise ValueError

    def load_catridge(self, catridge: QuantosCatridge):
        if (self._current_catridge == None):
            catridge.loaded_to_quantos = False
            self._current_catridge = None
        else:
            raise QuantosCatridgeLoadedError()

    def unload_current_catridge(self):
        if (self._current_catridge != None):
            for catridge in _catridges:
                if (catridge == self._current_catridge):
                    catridge.loaded = False
                    break
            self._current_catridge = None
        else:
            print('unloading catridge when no catridge is loaded!!!')

    def setStationOp(self, stationOp: StationOpDescriptor):
        if (stationOp.stationName == self.__class__):
            self._current_catridge.dispense(stationOp.solid)
        else:
            raise ValueError


''' ==== Station Operation Descriptors ==== '''

class QuantosDispenseOpDescriptor(StationOpDescriptor):
    def __init__(self, solid: Solid):
        super().__init__(stationName=QuantosSolidDispenserQS2.__class__.__name__)
        self._solid = solid

    @property
    def solid(self):
        return self._solid

# TODO you can have another op for loading catridge and doing stuff


''' ==== Station Output Descriptors ==== '''

class QuantosOutputDescriptor(StationOutputDescriptor):
    def __init__(self, opName: str, success:bool):
        super().__init__(opName=opName, succes=success)
