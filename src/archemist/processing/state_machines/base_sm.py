from typing import Dict
from archemist.state.station import Station
from archemist.util import Location

class BaseSm:
    def __init__(self, station: Station, params_dict: Dict) -> None:
        self._station = station
        self._current_batches_count = 0
        self._current_batch_index = -1
        self.machine = None

    def all_batches_assigned(self) -> bool:
        return not self._station.has_free_batch_capacity()

    def is_station_job_ready(self) -> bool:
        return not self._station.has_assigned_station_op() and not self._station.has_requested_robot_op()

    def are_all_batches_loaded(self):
        return self._current_batches_count == self._station.batch_capacity

    def update_loaded_batch(self):
        self.update_batch_loc_to_station()
        self._current_batches_count += 1
        if self._current_batches_count == self._station.batch_capacity:
            self._current_batch_index = 0
        else:
            self._current_batch_index += 1

    def update_batch_loc_to_station(self):
        self._station.assigned_batches[self._current_batch_index].location = self._station.location

    def are_all_batches_unloaded(self):
        return self._current_batches_count == 0

    def update_unloaded_batch(self):
        self.update_batch_loc_to_robot()
        self._current_batches_count -= 1
        if self._current_batches_count == self._station.batch_capacity:
            self._current_batch_index = 0
        else:
            self._current_batch_index += 1

    def update_batch_loc_to_robot(self):
        last_executed_robot_op = self._station.requested_robot_op_history[-1]
        self._station.assigned_batches[self._current_batch_index].location = Location(-1,-1,f'{last_executed_robot_op.robot_stamp}/Deck')
