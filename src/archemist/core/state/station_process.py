from __future__ import annotations
from bson.objectid import ObjectId
from transitions import Machine, State
from typing import Dict, List, Any, Union, Type
from archemist.core.persistence.models_proxy import ModelProxy, EmbedModelProxy,ListProxy, DictProxy
from archemist.core.persistence.object_factory import RobotOpFactory, StationOpFactory, ProcessFactory
from archemist.core.models.station_process_model import StationProcessModel, keyOpDetailsModel
from archemist.core.state.robot_op import RobotOpDescriptor
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.state.lot import Lot
from archemist.core.util.enums import ProcessStatus, OpOutcome


class keyOpDetails:
    def __init__(self, op_details_model: Union[keyOpDetailsModel, EmbedModelProxy]):
        self._model_proxy = op_details_model

    @classmethod
    def from_dict(cls, key_op_dict: Dict[str, Any]):
        model = keyOpDetailsModel()
        model.op_type = key_op_dict["type"]
        model.repeat_for_all_batches = key_op_dict["repeat_for_all_batches"]
        model.parameters = key_op_dict["parameters"]
        return cls(model)

    @property
    def model(self) -> keyOpDetailsModel:
        if isinstance(self._model_proxy, EmbedModelProxy):
            return self._model_proxy.model
        else:
            return self._model_proxy

    @property
    def op_type(self) -> str:
        return self._model_proxy.op_type

    @property
    def repeat_for_all_batches(self) -> bool:
        return self._model_proxy.repeat_for_all_batches

    @property
    def parameters(self) -> List[Dict]:
        return self._model_proxy.parameters

class StationProcess:
    TRIGGER_METHOD = '_process_state_transitions'
    
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        if isinstance(process_model, ModelProxy):
            self._model_proxy = process_model
        else:
            self._model_proxy = ModelProxy(process_model)
        
        self.STATES: List[State] = []
        self.TRANSITIONS: List[Dict[str, str]] = []
        self._state_machine = None

    @classmethod
    def _set_model_common_fields(cls, proc_model: StationProcessModel, associated_station: str,
                                 lot: Lot, key_op_dicts_list: List[Dict[str, Any]], **kwargs):
        proc_model._type = cls.__name__
        proc_model._module = cls.__module__
        proc_model.lot = lot.model
        proc_model.associated_station = associated_station
        
        if key_op_dicts_list:
            for key_op_dict in key_op_dicts_list:
                key_op_name = key_op_dict["name"]
                key_op_details = keyOpDetails.from_dict(key_op_dict)
                proc_model.key_ops_dict[key_op_name] = key_op_details.model

        proc_model.processing_slot = kwargs.get("processing_slot")        
        proc_model.skip_robot_ops = kwargs.get("skip_robot_ops", False)
        proc_model.skip_station_ops = kwargs.get("skip_station_ops", False)
        proc_model.skip_ext_procs =  kwargs.get("skip_ext_procs", False)

    @classmethod
    def from_args(cls, lot: Lot, key_op_dicts_list: List[Dict[str, Any]] = None, **kwargs):
        model = StationProcessModel()
        cls._set_model_common_fields(model, "Station", lot, key_op_dicts_list, **kwargs)
        model.save()
        return cls(model)
    
    @property
    def model(self) -> StationProcessModel:
        return self._model_proxy.model

    @property
    def object_id(self) -> ObjectId:
        return self._model_proxy.object_id
    
    @property
    def requested_by(self) -> ObjectId:
        return self._model_proxy.requested_by
    
    @requested_by.setter
    def requested_by(self, station_id: ObjectId):
        self._model_proxy.requested_by = station_id

    @property
    def associated_station(self) -> str:
        return self._model_proxy.associated_station
    
    @property
    def skip_robot_ops(self) -> bool:
        return self._model_proxy.skip_robot_ops
    
    @property
    def skip_station_ops(self) -> bool:
        return self._model_proxy.skip_station_ops
    
    @property
    def skip_ext_procs(self) -> bool:
        return self._model_proxy.skip_ext_procs

    @property
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
        return ListProxy(self._model_proxy.req_robot_ops, RobotOpFactory.create_from_model)
    
    @property
    def robot_ops_history(self) -> List[Type[RobotOpDescriptor]]:
        return ListProxy(self._model_proxy.robot_ops_history, RobotOpFactory.create_from_model)
    
    @property
    def key_ops_dict(self) -> Dict[str, keyOpDetails]:
        return DictProxy(self._model_proxy.key_ops_dict, keyOpDetails)
    
    @property
    def req_station_ops(self) -> List[Type[StationOpDescriptor]]:
        return ListProxy(self._model_proxy.req_station_ops, StationOpFactory.create_from_model)
    
    @property
    def station_ops_history(self) -> List[Type[StationOpDescriptor]]:
        return ListProxy(self._model_proxy.station_ops_history, StationOpFactory.create_from_model)
    
    @property
    def req_station_procs(self) -> List[Type[StationProcess]]:
        return ListProxy(self._model_proxy.req_station_procs, ProcessFactory.create_from_model)
    
    @property
    def station_procs_history(self) -> List[Type[StationProcess]]:
        return ListProxy(self._model_proxy.station_procs_history, ProcessFactory.create_from_model)

    def tick(self):
        if not self._state_machine:
            if self.status == ProcessStatus.INACTIVE:
                self._model_proxy.status = ProcessStatus.RUNNING
            self._state_machine = self._construct_state_machine()
        self._process_state_transitions()
        if self.m_state == "final_state":
            self._model_proxy.status = ProcessStatus.FINISHED
    
    def request_robot_ops(self, robot_ops: List[Type[RobotOpDescriptor]]):
        if not self.skip_robot_ops:
            for robot_op in robot_ops:
                self.req_robot_ops.append(robot_op)
            self._model_proxy.status = ProcessStatus.REQUESTING_ROBOT_OPS

    def are_req_robot_ops_completed(self) -> bool:
        req_robot_ops = [robot_op for robot_op in self.req_robot_ops]
        for robot_op in req_robot_ops:
            if robot_op.has_result and robot_op.outcome != OpOutcome.FAILED:
                self.req_robot_ops.remove(robot_op)
                self.robot_ops_history.append(robot_op)
        if len(self.req_robot_ops) == 0:
            self._model_proxy.status = ProcessStatus.RUNNING
            return True
        return False

    def create_key_op(self, key_op_name: str, batch_index: int=0) -> Type[StationOpDescriptor]:
        key_op_dtl = self.key_ops_dict[key_op_name]

        if not key_op_dtl.parameters:
            key_op = StationOpFactory.create_from_args(key_op_dtl.op_type)
        else:
            if key_op_dtl.repeat_for_all_batches and batch_index > 0:
                print(f"warning: non-zero index is specified for op with repeat_for_all_batches flag")
                print(f"warning: setting the index to zero")
                batch_index = 0
            key_op = StationOpFactory.create_from_args(key_op_dtl.op_type, key_op_dtl.parameters[batch_index])
        
        return key_op
    
    def request_station_op(self, station_op: Type[StationOpDescriptor]):
        if not self.skip_station_ops:
            self.req_station_ops.append(station_op)
            self._model_proxy.status = ProcessStatus.REQUESTING_STATION_OPS
    
    def are_req_station_ops_completed(self) -> bool:
        req_station_ops = [station_op for station_op in self.req_station_ops]
        for station_op in req_station_ops:
            if station_op.has_result and station_op.outcome != OpOutcome.FAILED:
                self.req_station_ops.remove(station_op)
                self.station_ops_history.append(station_op)
        if len(self.req_station_ops) == 0:
            self._model_proxy.status = ProcessStatus.RUNNING
            return True
        return False
    
    def request_station_process(self, station_process: Type[StationProcess]):
        if not self.skip_ext_procs:
            self.req_station_procs.append(station_process)
            self._model_proxy.status = ProcessStatus.REQUESTING_STATION_PROCS
    
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
    
    def switch_to_waiting(self):
        if self.status == ProcessStatus.REQUESTING_ROBOT_OPS:
            self._model_proxy.status = ProcessStatus.WAITING_ON_ROBOT_OPS
        elif self.status == ProcessStatus.REQUESTING_STATION_OPS:
            self._model_proxy.status = ProcessStatus.WAITING_ON_STATION_OPS
        elif self.status == ProcessStatus.REQUESTING_STATION_PROCS:
            self._model_proxy.status = ProcessStatus.WAITING_ON_STATION_PROCS
    
    def _construct_state_machine(self) -> Machine:
        states = self.STATES if self.STATES else [State(name="init_state"), State(name="final_state")]
        transitions = self.TRANSITIONS if self.TRANSITIONS else [{'source':'init_state','dest':'final_state'}]
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