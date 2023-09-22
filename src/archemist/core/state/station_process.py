from transitions import Machine, State
from typing import Dict, List, Any
from archemist.core.state.station import Station, StationProcessData
from archemist.core.state.batch import Batch
from archemist.core.util.location import Location

class StationProcess:
    TRIGGER_METHOD = 'process_state_transitions'
    
    def __init__(self, station: Station, process_data: StationProcessData, states: List[State], 
                 transitions: Dict, **kwargs) -> None:
        self._station = station
        self._process_data = process_data
        self._station.set_process_data(self._process_data)
        self.machine = self._construct_state_machine(states, transitions)

    @property
    def data(self) -> StationProcessData:
        return self._process_data
    
    def request_robot_op(self, robot_op, current_batch_id: int=-1):
        self._process_data.req_robot_ops.append(robot_op)
        self._station.request_robot_op(robot_op, current_batch_id)

    def request_station_op(self, station_op):
        self._process_data.req_station_ops.append(station_op)
        self._station.assign_station_op(station_op) # TODO replaced by request for external and assign for internal

    def are_req_robot_ops_completed(self) -> bool:
        for index, req_op in enumerate(self._process_data.req_robot_ops):
            complete_op = self._station.completed_robot_ops.get(str(req_op.uuid))
            if complete_op and complete_op.was_successful:
                self._process_data.req_robot_ops.pop(index)
                self._process_data.robot_ops_history.append(str(req_op.uuid))
                break
        return len(self._process_data.req_robot_ops) == 0
    
    def are_req_station_ops_completed(self) -> bool:
        for index,req_op in enumerate(self._process_data.req_station_ops):
            complete_op = self._station.completed_station_ops.get(str(req_op.uuid))
            if complete_op and complete_op.was_successful:
                self._process_data.req_station_ops.pop(index)
                self._process_data.station_ops_history.append(str(req_op.uuid))
                break
        return len(self._process_data.req_station_ops) == 0
    
    def _update_data_model(self):
        self._station.set_process_data(self._process_data)

    def _construct_state_machine(self, states: List[State], transitions: Dict) -> Machine:
        # add default trigger function to all transitions
        for transition in transitions:
            transition['trigger'] = self.TRIGGER_METHOD
        # add default entry callback to all states
        for state in states:
            state.add_callback('enter', self._default_entry_callback)
        return Machine(self, states=states, initial=self._process_data.status['state'], transitions=transitions)

    def _update_batch_loc_to_station(self, batch_index: int):
        batch = self._process_data.batches[batch_index]
        batch.location = self._station.location

    def _update_batch_loc_to_robot(self, batch_index: int):
        batch = self._process_data.batches[batch_index]
        last_robot_op_uuid = self._process_data.robot_ops_history[-1]
        last_robot_op = self._station.completed_robot_ops[last_robot_op_uuid]
        batch.location = Location(-1,-1,f'{last_robot_op.robot_stamp}/Deck')

    def _default_entry_callback(self):
        print(f'[{self.__class__.__name__}]: current state is {self.state}')
        self._process_data.status['state'] = self.state
        self._update_data_model()
