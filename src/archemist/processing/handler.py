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
            station_op = self.process()
            self.update_batch(station_op)
            self._station.state = StationState.PROCESSING_COMPLETE
        
        self._state.modifyObjectDB(self._station)

    def update_batch(self, operation_op):
        if (self._station_sm.batch_mode):
            for _ in range(0, self._station.assigned_batch.num_samples):
                self._station.assigned_batch.get_current_sample.add_opeation_op(operation_op)
                self._station.assigned_batch.process_current_sample()
        else:
            self._station.assigned_batch.get_current_sample.add_opeation_op(operation_op)
            self._station.assigned_batch.process_current_sample()