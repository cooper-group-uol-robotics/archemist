from archemist.state.state import State
from archemist.persistence.persistenceManager import Parser
from archemist.state.station import StationState


class StationHandler:
    def __init__(self, station_name: str):
        self._state = State()
        self._state.initializeState(False)
        self._station_name = station_name
        self._station = self._state.getStation(station_name)
        
        parser = Parser()
        self._station_sm = parser.create_process_sm(self._station_name)
        self._station_sm.set_station(self._station)

    def process(self):
        pass

    def handle(self):
        self._state.updateFromDB()
        self._station = self.state.getStation(self._station_name)

        self._station_sm.process_state_transitions()
        if (self._station.state == StationState.PROCESSING):
            self.process()
            self._station.state = StationState.PROCESSING_COMPLETE
        
        self._state.modifyObjectDB(self._station)