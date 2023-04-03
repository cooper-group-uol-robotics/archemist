# External
from transitions import Machine, State
from typing import Dict, List

# Core
from archemist.core.state.station import Station
from archemist.core.util import Location


class StationProcessFSM:
    def __init__(self, station: Station, params_dict: Dict) -> None:
        self._station = station
        self.machine = None
        self._trigger_function = "process_state_transitions"
        self._status = {}
        status = self._station.process_status
        if status:
            self._status = status
        else:
            self._status["state"] = "init_state"
            self._status["batches_count"] = 0
            self._status["batch_index"] = 0

    def init_state_machine(self, states: List[State], transitions: Dict):
        # add default entry callback to all states
        for state in states:
            state.add_callback("enter", self._default_entry_callback)
        self.machine = Machine(
            self, states=states, initial=self._status["state"], transitions=transitions
        )

    def all_batches_assigned(self) -> bool:
        return not self._station.has_free_batch_capacity()

    def is_station_job_ready(self) -> bool:
        return (
            not self._station.has_assigned_station_op()
            and not self._station.has_requested_robot_op()
        )

    def are_all_batches_loaded(self):
        return self._status["batches_count"] == self._station.batch_capacity

    def update_loaded_batch(self):
        self.update_batch_loc_to_station()
        self._status["batches_count"] += 1
        if self._status["batches_count"] == self._station.batch_capacity:
            self._status["batch_index"] = 0
        else:
            self._status["batch_index"] += 1

    def update_batch_loc_to_station(self):
        self._station.assigned_batches[
            self._status["batch_index"]
        ].location = self._station.location

    def are_all_batches_unloaded(self):
        return self._status["batches_count"] == 0

    def update_unloaded_batch(self):
        self.update_batch_loc_to_robot()
        self._status["batches_count"] -= 1
        if self._status["batches_count"] == self._station.batch_capacity:
            self._status["batch_index"] = 0
        else:
            self._status["batch_index"] += 1

    def update_batch_loc_to_robot(self):
        last_executed_robot_op = self._station.requested_robot_op_history[-1]
        self._station.assigned_batches[self._status["batch_index"]].location = Location(
            -1, -1, f"{last_executed_robot_op.robot_stamp}/Deck"
        )

    def _default_entry_callback(self):
        print(f"[{self.__class__.__name__}]: current state is {self.state}")
        self._status["state"] = self.state
        self._station.process_status = self._status
