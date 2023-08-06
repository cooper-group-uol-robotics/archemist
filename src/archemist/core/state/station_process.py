from __future__ import annotations
import uuid
from bson.objectid import ObjectId
from transitions import Machine, State
from typing import Dict, List, Any, Union, Type
from archemist.core.persistence.models_proxy import ModelProxy, ListProxy
from archemist.core.persistence.object_factory import RobotFactory, StationFactory, ProcessFactory
from archemist.core.models.station_process_model import StationProcessModel
from archemist.core.state.robot_op import RobotOpDescriptor
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.state.lot import Lot
from archemist.core.util.location import Location
from archemist.core.util.enums import ProcessStatus

class StationProcess:
    TRIGGER_METHOD = '_process_state_transitions'
    STATES: List[State] = []
    TRANSITIONS: List[Dict[str, str]] = []
    
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        if isinstance(process_model, ModelProxy):
            self._model_proxy = process_model
        else:
            self._model_proxy = ModelProxy(process_model)
        
        self._state_machine = None

    @classmethod
    def _set_model_common_fields(cls, proc_model: StationProcessModel, associated_station: str,
                                 lot: Lot, key_process_ops: List[Type[StationOpDescriptor]],
                                 processing_slot: int = None, **kwargs):
        proc_model.uuid = uuid.uuid4()
        proc_model.lot = lot.model
        proc_model.key_process_ops = [key_process_op.model for key_process_op in key_process_ops]
        if processing_slot:
            proc_model.processing_slot = processing_slot
        proc_model.associated_station = associated_station

    @classmethod
    def from_args(cls, lot: Lot, key_process_ops: List[Type[StationOpDescriptor]], processing_slot: int = None):
        model = StationProcessModel()
        cls._set_model_common_fields(model, "Station", lot, key_process_ops, processing_slot)
        model._type = cls.__name__
        model._module = cls.__module__
        model.save()
        return cls(model)
    
    @property
    def model(self) -> StationProcessModel:
        return self._model_proxy.model

    @property
    def uuid(self) -> uuid.UUID:
        return self._model_proxy.uuid
    
    @property
    def requested_by(self) -> ObjectId:
        return self._model_proxy.requested_by
    
    @requested_by.setter
    def requested_by(self, station_id: ObjectId):
        self._model_proxy.requested_by = station_id

    @property
    def associated_station(self) -> str:
        return self._model_proxy.associated_station
    
    @ property
    def status(self) -> ProcessStatus:
        return self._model_proxy.status
    
    @property
    def m_state(self) -> str:
        return self._model_proxy.state
    
    @property
    def processing_slot(self) -> int:
        return self._model_proxy.processing_slot
    
    @processing_slot.setter
    def processing_slot(self, new_slot: int):
        self._model_proxy.processing_slot = new_slot
    
    @property
    def data(self) -> Dict[str, Any]:
        return self._model_proxy.data

    @property
    def lot(self) -> Lot:
        return Lot(self._model_proxy.lot)
    
    @property
    def req_robot_ops(self) -> List[Type[RobotOpDescriptor]]:
        return ListProxy(self._model_proxy.req_robot_ops, RobotFactory.create_op_from_model)
    
    @property
    def robot_ops_history(self) -> List[Type[RobotOpDescriptor]]:
        return ListProxy(self._model_proxy.robot_ops_history, RobotFactory.create_op_from_model)
    
    @property
    def key_process_ops(self) -> List[Type[StationOpDescriptor]]:
        return ListProxy(self._model_proxy.key_process_ops, StationFactory.create_op_from_model)
    
    @property
    def req_station_ops(self) -> List[Type[StationOpDescriptor]]:
        return ListProxy(self._model_proxy.req_station_ops, StationFactory.create_op_from_model)
    
    @property
    def station_ops_history(self) -> List[Type[StationOpDescriptor]]:
        return ListProxy(self._model_proxy.station_ops_history, StationFactory.create_op_from_model)
    
    @property
    def req_station_procs(self) -> List[Type[StationProcess]]:
        return ListProxy(self._model_proxy.req_station_procs, ProcessFactory.create_process_from_model)
    
    @property
    def station_procs_history(self) -> List[Type[StationProcess]]:
        return ListProxy(self._model_proxy.station_procs_history, ProcessFactory.create_process_from_model)

    def tick(self):
        if not self._state_machine:
            self._model_proxy.status = ProcessStatus.RUNNING
            self._state_machine = self._construct_state_machine()
        self._process_state_transitions()
        if self.m_state == "final_state":
            self._model_proxy.status = ProcessStatus.FINISHED
    
    def request_robot_op(self, robot_op: Type[RobotOpDescriptor]):
        self.req_robot_ops.append(robot_op)
        self._model_proxy.status = ProcessStatus.WAITING_ON_ROBOT_OPS

    def are_req_robot_ops_completed(self) -> bool:
        req_robot_ops = [robot_op for robot_op in self.req_robot_ops]
        for robot_op in req_robot_ops:
            if robot_op.has_result and robot_op.was_successful:
                self.req_robot_ops.remove(robot_op)
                self.robot_ops_history.append(robot_op)
        if len(self.req_robot_ops) == 0:
            self._model_proxy.status = ProcessStatus.RUNNING
            return True
        return False
    
    def request_station_op(self, station_op: Type[StationOpDescriptor]):
        self.req_station_ops.append(station_op)
        self._model_proxy.status = ProcessStatus.WAITING_ON_STATION_OPS
    
    def are_req_station_ops_completed(self) -> bool:
        req_station_ops = [station_op for station_op in self.req_station_ops]
        for station_op in req_station_ops:
            if station_op.has_result and station_op.was_successful:
                self.req_station_ops.remove(station_op)
                self.station_ops_history.append(station_op)
        if len(self.req_station_ops) == 0:
            self._model_proxy.status = ProcessStatus.RUNNING
            return True
        return False
    
    def request_station_process(self, station_process: Type[StationProcess]):
        self.req_station_procs.append(station_process)
        self._model_proxy.status = ProcessStatus.WAITING_ON_STATION_PROCS
    
    def are_req_station_procs_completed(self) -> bool:
        req_station_procs = [station_proc for station_proc in self.req_station_procs]
        for station_proc in req_station_procs:
            if station_proc.status == ProcessStatus.FINISHED:
                self.req_station_procs.remove(station_proc)
                self.station_procs_history.append(station_proc)
        if len(self.req_station_procs) == 0:
            self._model_proxy.status = ProcessStatus.RUNNING
            return True
        return False
    
    def _construct_state_machine(self) -> Machine:
        states = self.STATES
        transitions = self.TRANSITIONS
        # add default trigger function to all transitions
        for transition in transitions:
            transition['trigger'] = self.TRIGGER_METHOD
        # add default entry callback to all states
        for state in states:
            state.add_callback('enter', self._default_entry_callback)
        return Machine(self, states=states, initial=self.m_state, transitions=transitions)

    def _default_entry_callback(self):
        self._model_proxy.state = self.state
        print(f'[{self.__class__.__name__}]: current state is {self.m_state}')

    def _process_state_transitions(self):
        pass