from typing import List, Any, Dict, Union, Type
from archemist.core.persistence.models_proxy import ModelProxy, ListProxy
from archemist.core.util.enums import StationState, OpState
from archemist.core.models.station_model import StationModel
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.state.material import Liquid,Solid
from archemist.core.util.location import Location
from archemist.core.state.robot import RobotOpDescriptor
from archemist.core.state.lot import Lot
from archemist.core.state.station_process import StationProcess
from archemist.core.persistence.object_factory import StationFactory, RobotFactory, ProcessFactory
from bson.objectid import ObjectId
import uuid

class Station:
    def __init__(self, station_model: Union[Type[StationModel], ModelProxy]) -> None:
        if isinstance(station_model, ModelProxy):
            self._model_proxy = station_model
        else:
            self._model_proxy = ModelProxy(station_model)

    @classmethod
    def from_dict(cls, station_dict: dict, liquids: List[Liquid] = None, solids: List[Solid] = None):
        model = StationModel()
        cls._set_model_common_fields(station_dict, liquids, solids, model)
        model._module = cls.__module__
        model.save()
        return cls(model)

    @staticmethod
    def _set_model_common_fields(station_dict: dict, liquids: List[Liquid], solids: List[Solid], station_model: StationModel):
        station_model._type = station_dict['type']
        station_model.exp_id = station_dict['id']
        station_model.location = station_dict['location']
        station_model.total_batch_capacity = station_dict['total_batch_capacity']
        station_model.process_batch_capacity = station_dict['process_batch_capacity']
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
    def state(self) -> StationState:
        return self._model_proxy.state

    @property
    def id(self) -> int:
        return self._model_proxy.exp_id

    @property
    def location(self) -> Location:
        loc_dict = self._model_proxy.location
        return Location(node_id=loc_dict['node_id'],graph_id=loc_dict['graph_id'], frame_name='')
    
    def get_handler_details(self) -> Dict[str, str]:
        station_module = self._model_proxy._module.rsplit('.',1)[0]
        return {'type':self._model_proxy.selected_handler, 'module':station_module}

    ''' batch capacity '''

    @property
    def total_batch_capacity(self) -> int:
        return self._model_proxy.total_batch_capacity
    
    @property
    def free_batch_capacity(self) -> int:
        num_current_batches = sum([lot.num_batches for lot in self._assigned_lots])
        return self.total_batch_capacity - num_current_batches
    
    @property
    def process_batch_capacity(self) -> int:
        return self._model_proxy.process_batch_capacity
    
    ''' Process properties and methods '''
    
    @property
    def requested_ext_procs(self) -> List[Type[StationProcess]]:
        return ListProxy(self._model_proxy.requested_ext_procs, ProcessFactory.create_process_from_model)
    
    @property
    def queued_procs(self) -> List[Type[StationProcess]]:
        return ListProxy(self._model_proxy.queued_procs, ProcessFactory.create_process_from_model)
    
    @property
    def running_procs(self) -> List[Type[StationProcess]]:
        return ListProxy(self._model_proxy.running_procs, ProcessFactory.create_process_from_model)
    
    @property
    def completed_procs(self) -> List[Type[StationProcess]]:
        return ListProxy(self._model_proxy.completed_procs, ProcessFactory.create_process_from_model)
    
    def request_external_process(self, ext_proc: Type[StationProcess]):
        self.requested_ext_procs.append(ext_proc)

    ''' Lots properties and methods '''

    @property
    def _assigned_lots(self) -> List[Lot]:
        return ListProxy(self._model_proxy.assigned_lots, Lot)

    @property
    def processed_lots(self) -> List[Lot]:
        return ListProxy(self._model_proxy.processed_lots, Lot)

    def assign_lot(self, lot: Lot):
        if self.free_batch_capacity >= lot.num_batches:
            lot.add_station_stamp(str(self))
            self._assigned_lots.append(lot)
            self._log_station(f'{lot} is added for processing')
        else:
            self._log_station(f'Cannot add {lot}, no batch capacity is available')
    
    def finish_processing_lot(self, lot: Lot):
        lot.add_station_stamp(str(self))
        self._assigned_lots.remove(lot)
        self.processed_lots.append(lot)
        self._log_station(f'processing {lot} is complete')
        
    ''' Robot ops properties '''

    @property
    def requested_robot_ops(self) -> List[Type[RobotOpDescriptor]]:
        return ListProxy(self._model_proxy.requested_robot_ops, RobotFactory.create_op_from_model)

    ''' Station ops properties and methods '''

    @property
    def _queued_ops(self) -> List[Type[StationOpDescriptor]]:
        return ListProxy(self._model_proxy.queued_ops, StationFactory.create_op_from_model)

    @property
    def assigned_op(self) -> Type[StationOpDescriptor]:
        return StationFactory.create_op_from_model(self._model_proxy.assigned_op) \
               if self._model_proxy.assigned_op else None
    
    @property
    def assigned_op_state(self) -> OpState:
        return self._model_proxy.assigned_op_state
    
    @assigned_op_state.setter
    def assigned_op_state(self, new_state: OpState):
        self._model_proxy.assigned_op_state = new_state

    @property
    def completed_ops(self) -> List[Type[StationOpDescriptor]]:
        return ListProxy(self._model_proxy.completed_ops, StationFactory.create_op_from_model)
    
    def update_assigned_op(self):
        if self._queued_ops and self.assigned_op is None:
            op = self._queued_ops.pop()
            self._model_proxy.assigned_op = op.model
            self.assigned_op_state = OpState.ASSIGNED

    def add_station_op(self, station_op: Type[StationOpDescriptor]):
        if station_op.requested_by is None:
            station_op.requested_by = self.object_id
        self._queued_ops.append(station_op)
        self._log_station(f'{station_op} is added to queued_op list')  

    def complete_assigned_op(self, success: bool, **kwargs):
        op = self.assigned_op
        if op:
            op.complete_op(success, **kwargs)
            self._model_proxy.assigned_op = None
            self.assigned_op_state = OpState.INVALID
            self.completed_ops.append(op)
            self._log_station(f'{op} is complete')

    def repeat_assigned_op(self):
        if self.assigned_op:
            self.assigned_op_state = OpState.TO_BE_REPEATED
        else:
            self._log_station('Unable to repeat. No op assigned')

    def skip_assigned_op(self):
        if self.assigned_op:
            self.assigned_op_state = OpState.TO_BE_SKIPPED
        else:
            self._log_station('Unable to skip. No op assigned')

    def _log_station(self, message: str):
        print(f'[{self}]: {message}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}_{self.id}'


