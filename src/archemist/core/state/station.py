from __future__ import annotations
from typing import List, Dict, Union, Type, TYPE_CHECKING
if TYPE_CHECKING:
    from archemist.core.state.station_process import StationProcess
    from archemist.core.state.robot import RobotOp
    from archemist.core.state.station_op import StationOp
    from archemist.core.state.station_op_result import StationOpResult

from archemist.core.persistence.models_proxy import ModelProxy, ListProxy, DictProxy
from archemist.core.util.enums import StationState, OpState, OpOutcome, LotStatus
from archemist.core.models.station_model import StationModel
from archemist.core.state.material import Liquid,Solid
from archemist.core.util.location import Location
from archemist.core.state.lot import Lot
from archemist.core.persistence.object_factory import StationOpFactory, RobotOpFactory, ProcessFactory
from bson.objectid import ObjectId

class Station:
    def __init__(self, station_model: Union[Type[StationModel], ModelProxy]) -> None:
        if isinstance(station_model, ModelProxy):
            self._model_proxy = station_model
        else:
            self._model_proxy = ModelProxy(station_model)
        
        if self.liquids_dict:
            for liquid in self.liquids_dict.values():
                liquid.belongs_to = self.object_id
        
        if self.solids_dict:
            for solid in self.solids_dict.values():
                solid.belongs_to = self.object_id

    @classmethod
    def from_dict(cls, station_dict: dict):
        model = StationModel()
        cls._set_model_common_fields(model, station_dict)
        model.save()
        return cls(model)

    @classmethod
    def _set_model_common_fields(cls, station_model: StationModel, station_dict: dict):
        station_model._type = station_dict['type']
        station_model._module = cls.__module__
        station_model.exp_id = station_dict['id']
        station_model.location = Location.from_dict(station_dict['location']).model
        station_model.total_lot_capacity = station_dict['total_lot_capacity']
        slots_num = station_dict['total_lot_capacity']
        station_model.lot_slots = {str(slot_num): None for slot_num in range(slots_num)}
        station_model.selected_handler = station_dict['handler']

        materials_dict = station_dict.get('materials', {})
        if materials_dict:
            if 'liquids' in materials_dict:
                for liquid_dict in materials_dict['liquids']:
                    # construct liquid
                    liquid = Liquid.from_dict(liquid_dict)
                    # add material to liquids_dict
                    station_model.liquids_dict[liquid.name] = liquid.object_id
            
            if 'solids' in materials_dict:
                for solid_dict in materials_dict['solids']:
                    # construct solid
                    solid = Solid.from_dict(solid_dict)
                    # add material to liquids_dict
                    station_model.solids_dict[solid.name] = solid.object_id

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
        return Location(self._model_proxy.location)
    
    @property
    def selected_handler(self) -> str:
        return self._model_proxy.selected_handler

    ''' materials '''
    @property
    def liquids_dict(self) -> Dict[str, Liquid]:
        if self._model_proxy.liquids_dict:
            return DictProxy(self._model_proxy.liquids_dict, Liquid.from_object_id)
    
    @property
    def solids_dict(self) -> Dict[str, Solid]:
        if self._model_proxy.solids_dict:
            return DictProxy(self._model_proxy.solids_dict, Solid.from_object_id)
    
    ''' lot capacity '''

    @property
    def total_lot_capacity(self) -> int:
        return self._model_proxy.total_lot_capacity
    
    @property
    def free_lot_capacity(self) -> int:
        free_lot_capacity = 0
        for lot in self.lot_slots.values():
            if not lot:
                free_lot_capacity += 1

        return free_lot_capacity
    
    ''' Process properties and methods '''
    
    @property
    def requested_ext_procs(self) -> List[Type[StationProcess]]:
        return ListProxy(self._model_proxy.requested_ext_procs, ProcessFactory.create_from_model)
    
    @property
    def queued_procs(self) -> List[Type[StationProcess]]:
        return ListProxy(self._model_proxy.queued_procs, ProcessFactory.create_from_model)
    
    @property
    def running_procs(self) -> List[Type[StationProcess]]:
        return ListProxy(self._model_proxy.running_procs, ProcessFactory.create_from_model)

    @property
    def num_running_main_procs(self) -> int:
        num_main_procs = 0
        for proc in self.running_procs:
            num_main_procs += 1 if not proc.is_subprocess else 0
        return num_main_procs
    
    @property
    def procs_history(self) -> List[Type[StationProcess]]:
        return ListProxy(self._model_proxy.procs_history, ProcessFactory.create_from_model)
    
    def request_external_process(self, ext_proc: Type[StationProcess]):
        ext_proc.requested_by = self.object_id
        self.requested_ext_procs.append(ext_proc)

    def add_process(self, proc: Type[StationProcess]):
        self.queued_procs.append(proc)

    ''' Lots properties and methods '''

    @property
    def lot_slots(self) -> Dict[str, Lot]:
        # to handle empty slots with None value
        modified_constructor = lambda model: Lot.from_object_id(model.object_id) if model else None
        return DictProxy(self._model_proxy.lot_slots, modified_constructor)

    def add_lot(self, added_lot: Lot):
        lot_added = False
        for slot, lot in self.lot_slots.items():
            if lot is None:
                lot_added = True
                added_lot.add_station_stamp(str(self))
                self.lot_slots[slot] = added_lot
                self._log_station(f'{added_lot} is added for processing at slot {slot}')
                break
       
        if not lot_added:
            self._log_station(f'Cannot add {added_lot}, no lot capacity is available')
    
    def finish_processing_lot(self, lot: Lot):
        lot.add_station_stamp(str(self))
        lot.status = LotStatus.READY_FOR_COLLECTION
        self._log_station(f'processing {lot} is complete')

    def is_lot_onboard(self, lot: Lot):
        onboard_lots = [lot for lot in self.lot_slots.values() if lot is not None]
        return lot in onboard_lots
    
    def retrieve_ready_for_collection_lots(self) -> List[Lot]:
        ready_for_collection_lots = []
        for slot, lot in self.lot_slots.items():
            if lot and lot.status == LotStatus.READY_FOR_COLLECTION:
                ready_for_collection_lots.append(lot)
                self.lot_slots[slot] = None
        
        return ready_for_collection_lots
    
    def has_ready_for_collection_lots(self) -> bool:
        for lot in self.lot_slots.values():
            if lot and lot.status == LotStatus.READY_FOR_COLLECTION:
                return True
        return False
        
    ''' Robot ops properties '''

    @property
    def requested_robot_ops(self) -> List[Type[RobotOp]]:
        return ListProxy(self._model_proxy.requested_robot_ops, RobotOpFactory.create_from_model)

    def add_req_robot_op(self, robot_op: Type[RobotOp]):
        robot_op.requested_by = self.object_id
        self.requested_robot_ops.append(robot_op)


    ''' Station ops properties and methods '''

    @property
    def _queued_ops(self) -> List[Type[StationOp]]:
        return ListProxy(self._model_proxy.queued_ops, StationOpFactory.create_from_model)

    @property
    def assigned_op(self) -> Type[StationOp]:
        return StationOpFactory.create_from_model(self._model_proxy.assigned_op) \
               if self._model_proxy.assigned_op else None
    
    @property
    def assigned_op_state(self) -> OpState:
        return self._model_proxy.assigned_op_state
    
    @property
    def ops_history(self) -> List[Type[StationOp]]:
        return ListProxy(self._model_proxy.ops_history, StationOpFactory.create_from_model)
    
    def update_assigned_op(self):
        if self._queued_ops and self.assigned_op is None:
            op = self._queued_ops.pop()
            self._model_proxy.assigned_op = op.model
            self._model_proxy.assigned_op_state = OpState.ASSIGNED

    def add_station_op(self, station_op: Type[StationOp]):
        if station_op.requested_by is None:
            station_op.requested_by = self.object_id
        self._queued_ops.append(station_op)
        self._log_station(f'{station_op} is added to queued_op list')

    def set_assigned_op_to_execute(self):
        self.assigned_op.add_start_timestamp()
        self._model_proxy.assigned_op_state = OpState.EXECUTING

    def complete_assigned_op(self, outcome: OpOutcome, results: List[Type[StationOpResult]]):
        op = self.assigned_op
        if op:
            op.complete_op(outcome, results)
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


