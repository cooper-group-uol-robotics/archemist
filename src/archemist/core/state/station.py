from typing import List, Dict, Union, Type
from archemist.core.persistence.models_proxy import ModelProxy, ListProxy, DictProxy
from archemist.core.util.enums import StationState, OpState, OpOutcome
from archemist.core.models.station_model import StationModel
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.state.material import Liquid,Solid
from archemist.core.util.location import Location
from archemist.core.state.robot import RobotOpDescriptor
from archemist.core.state.lot import Lot
from archemist.core.state.station_process import StationProcess
from archemist.core.persistence.object_factory import StationOpFactory, RobotOpFactory, ProcessFactory
from bson.objectid import ObjectId

class Station:
    def __init__(self, station_model: Union[Type[StationModel], ModelProxy]) -> None:
        if isinstance(station_model, ModelProxy):
            self._model_proxy = station_model
        else:
            self._model_proxy = ModelProxy(station_model)

    @classmethod
    def from_dict(cls, station_dict: dict, liquids: List[Liquid] = None, solids: List[Solid] = None):
        model = StationModel()
        cls._set_model_common_fields(model, station_dict, liquids, solids)
        model.save()
        return cls(model)

    @classmethod
    def _set_model_common_fields(cls, station_model: StationModel, station_dict: dict, liquids: List[Liquid], solids: List[Solid]):
        station_model._type = station_dict['type']
        station_model._module = cls.__module__
        station_model.exp_id = station_dict['id']
        station_model.location = station_dict['location']
        station_model.total_lot_capacity = station_dict['total_lot_capacity']
        proc_slots_num = station_dict['total_lot_capacity']
        station_model.running_procs_slots = {str(slot_num): None for slot_num in range(proc_slots_num)}
        station_model.selected_handler = station_dict['handler']
        if liquids:
            station_model.liquids = [liquid.model for liquid in liquids]
        if solids:
            station_model.solids = [solid.model for solid in solids]

    ''' General properties and methods'''

    @property
    def model(self) -> Type[StationModel]:
        return self._model_proxy.model
    
    @property
    def object_id(self) -> ObjectId:
        return self._model_proxy.object_id
    
    @property
    def module_path(self) -> str:
        return self._model_proxy._module

    @property
    def state(self) -> StationState:
        return self._model_proxy.state
    
    @state.setter
    def state(self, new_state: StationState):
        self._model_proxy.state = new_state

    @property
    def id(self) -> int:
        return self._model_proxy.exp_id

    @property
    def location(self) -> Location:
        loc_dict = self._model_proxy.location
        return Location(node_id=loc_dict['node_id'],graph_id=loc_dict['graph_id'], frame_name='')
    
    @property
    def selected_handler(self) -> str:
        return self._model_proxy.selected_handler

    ''' materials '''
    @property
    def liquids(self) -> List[Liquid]:
        if self._model_proxy.liquids:
            return ListProxy(self._model_proxy.liquids, Liquid)
    
    @property
    def solids(self) -> List[Solid]:
        if self._model_proxy.solids:
            return ListProxy(self._model_proxy.solids, Solid)
    
    ''' lot capacity '''

    @property
    def total_lot_capacity(self) -> int:
        return self._model_proxy.total_lot_capacity
    
    @property
    def free_lot_capacity(self) -> int:
        return self.total_lot_capacity - len(self.assigned_lots) - len(self.processed_lots)
    
    ''' Process properties and methods '''
    
    @property
    def requested_ext_procs(self) -> List[Type[StationProcess]]:
        return ListProxy(self._model_proxy.requested_ext_procs, ProcessFactory.create_from_model)
    
    @property
    def queued_procs(self) -> List[Type[StationProcess]]:
        return ListProxy(self._model_proxy.queued_procs, ProcessFactory.create_from_model)
    
    @property
    def running_procs_slots(self) -> Dict[int, Type[StationProcess]]:
        # to handle empty slots with None value
        modified_constructor = lambda model: ProcessFactory.create_from_object_id(model.object_id) if model else None
        return DictProxy(self._model_proxy.running_procs_slots, modified_constructor)
    
    @property
    def procs_history(self) -> List[Type[StationProcess]]:
        return ListProxy(self._model_proxy.procs_history, ProcessFactory.create_from_model)
    
    def request_external_process(self, ext_proc: Type[StationProcess]):
        self.requested_ext_procs.append(ext_proc)

    def add_process(self, proc: Type[StationProcess]):
        self.queued_procs.append(proc)

    ''' Lots properties and methods '''

    @property
    def assigned_lots(self) -> List[Lot]:
        return ListProxy(self._model_proxy.assigned_lots, Lot)

    @property
    def processed_lots(self) -> List[Lot]:
        return ListProxy(self._model_proxy.processed_lots, Lot)

    def add_lot(self, lot: Lot):
        if self.free_lot_capacity > 0:
            lot.add_station_stamp(str(self))
            self.assigned_lots.append(lot)
            self._log_station(f'{lot} is added for processing')
        else:
            self._log_station(f'Cannot add {lot}, no batch capacity is available')
    
    def finish_processing_lot(self, lot: Lot):
        lot.add_station_stamp(str(self))
        self.assigned_lots.remove(lot)
        self.processed_lots.append(lot)
        self._log_station(f'processing {lot} is complete')
        
    ''' Robot ops properties '''

    @property
    def requested_robot_ops(self) -> List[Type[RobotOpDescriptor]]:
        return ListProxy(self._model_proxy.requested_robot_ops, RobotOpFactory.create_from_model)

    def add_req_robot_op(self, robot_op: Type[RobotOpDescriptor]):
        robot_op.requested_by = self.object_id
        self.requested_robot_ops.append(robot_op)


    ''' Station ops properties and methods '''

    @property
    def _queued_ops(self) -> List[Type[StationOpDescriptor]]:
        return ListProxy(self._model_proxy.queued_ops, StationOpFactory.create_from_model)

    @property
    def assigned_op(self) -> Type[StationOpDescriptor]:
        return StationOpFactory.create_from_model(self._model_proxy.assigned_op) \
               if self._model_proxy.assigned_op else None
    
    @property
    def assigned_op_state(self) -> OpState:
        return self._model_proxy.assigned_op_state
    
    @property
    def ops_history(self) -> List[Type[StationOpDescriptor]]:
        return ListProxy(self._model_proxy.ops_history, StationOpFactory.create_from_model)
    
    def update_assigned_op(self):
        if self._queued_ops and self.assigned_op is None:
            op = self._queued_ops.pop()
            self._model_proxy.assigned_op = op.model
            self._model_proxy.assigned_op_state = OpState.ASSIGNED

    def add_station_op(self, station_op: Type[StationOpDescriptor]):
        if station_op.requested_by is None:
            station_op.requested_by = self.object_id
        self._queued_ops.append(station_op)
        self._log_station(f'{station_op} is added to queued_op list')

    def set_assigned_op_to_execute(self):
        self.assigned_op.add_start_timestamp()
        self._model_proxy.assigned_op_state = OpState.EXECUTING

    def complete_assigned_op(self, outcome: OpOutcome, **kwargs):
        op = self.assigned_op
        if op:
            op.complete_op(outcome, **kwargs)
            self._model_proxy.assigned_op = None
            self._model_proxy.assigned_op_state = OpState.INVALID
            self.ops_history.append(op)
            self._log_station(f'{op} is complete')

    def repeat_assigned_op(self):
        if self.assigned_op:
            self._model_proxy.assigned_op_state = OpState.TO_BE_REPEATED
        else:
            self._log_station('Unable to repeat. No op assigned')

    def skip_assigned_op(self):
        if self.assigned_op:
            self._model_proxy.assigned_op_state = OpState.TO_BE_SKIPPED
        else:
            self._log_station('Unable to skip. No op assigned')

    def _log_station(self, message: str):
        print(f'[{self}]: {message}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}_{self.id}'


