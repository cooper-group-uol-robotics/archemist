from typing import Dict, List, Union, Type

from archemist.core.state.robot_op import RobotOpDescriptor
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.state.recipe import Recipe
from archemist.core.state.station_process import StationProcess
from archemist.core.persistence.object_factory import RobotOpFactory, ProcessFactory
from archemist.core.models.state_model import WorkflowStateModel, InputStateModel, OutputStateModel
from archemist.core.persistence.models_proxy import ModelProxy, ListProxy, DictProxy
from archemist.core.util.enums import LotStatus
from archemist.core.util.location import Location

class WorkflowState:
    def __init__(self, state_model: Union[WorkflowStateModel, ModelProxy]):
        if isinstance(state_model, ModelProxy):
            self._model_proxy = state_model
        else:
            self._model_proxy = ModelProxy(state_model)

    @classmethod
    def from_args(cls, workflow_name: str):
        model = WorkflowStateModel()
        model.workflow_name = workflow_name
        model.save()
        return cls(model)

    @property
    def workflow_name(self) -> str:
        return self._model_proxy.workflow_name
    
    @property
    def lots_buffer(self) -> List[Lot]:
        return ListProxy(self._model_proxy.lots_buffer, Lot)
        
    @property
    def robot_ops_queue(self) -> List[Type[RobotOpDescriptor]]:
        return ListProxy(self._model_proxy.robot_ops_queue, RobotOpFactory.create_from_model)
        
    @property
    def proc_requests_queue(self) -> List[Type[StationProcess]]:
        return ListProxy(self._model_proxy.proc_requests_queue, ProcessFactory.create_from_model)
    
    
class InputState:
    def __init__(self, state_model: Union[InputStateModel, ModelProxy]):
        if isinstance(state_model, ModelProxy):
            self._model_proxy = state_model
        else:
            self._model_proxy = ModelProxy(state_model)

    @classmethod
    def from_dict(cls, state_dict: dict={}):
        model = InputStateModel()
        model.location = state_dict['location']
        model.samples_per_batch = state_dict['samples_per_batch']
        model.batches_per_lot = state_dict['batches_per_lot']
        model.total_lot_capacity = proc_slots_num = state_dict['total_lot_capacity']
        model.lot_input_process = state_dict["lot_input_process"]
        slots = {str(slot_num): None for slot_num in range(proc_slots_num)}
        model.lot_slots = slots
        model.proc_slots = slots
        model.save()
        return cls(model)
    
    @property
    def location(self) -> Location:
        loc_dict = self._model_proxy.location
        return Location(node_id=loc_dict['node_id'],graph_id=loc_dict['graph_id'], frame_name='')
    
    @property
    def samples_per_batch(self) -> int:
        return self._model_proxy.samples_per_batch
    
    @property
    def batches_per_lot(self) -> int:
        return self._model_proxy.batches_per_lot
    
    @property
    def total_lot_capacity(self) -> int:
        return self._model_proxy.total_lot_capacity

    @property
    def lot_input_process(self) -> Dict:
        return self._model_proxy.lot_input_process
    
    @property
    def batches_queue(self) -> List[Batch]:
        return ListProxy(self._model_proxy.batches_queue, Batch)
    
    @property
    def recipes_queue(self) -> List[Recipe]:
        return ListProxy(self._model_proxy.recipes_queue, Recipe)
    
    @property
    def requested_robot_ops(self) -> List[Type[RobotOpDescriptor]]:
        return ListProxy(self._model_proxy.requested_robot_ops, RobotOpFactory.create_from_model)
    
    @property
    def lot_slots(self) -> Dict[str, Lot]:
        # to handle empty slots with None value
        modified_constructor = lambda model: Lot.from_object_id(model.object_id) if model else None
        return DictProxy(self._model_proxy.lot_slots, modified_constructor)
    
    @property
    def proc_slots(self) -> Dict[str, Type[StationProcess]]:
        # to handle empty slots with None value
        modified_constructor = lambda model: ProcessFactory.create_from_object_id(model.object_id) if model else None
        return DictProxy(self._model_proxy.proc_slots, modified_constructor)

    @property
    def procs_history(self) -> List[Type[StationProcess]]:
        return ListProxy(self._model_proxy.procs_history, ProcessFactory.create_from_model)

    def get_lots_num(self, status: LotStatus=None) -> int:
        lots_num = 0
        for _, lot in self.lot_slots.items():
            if lot is not None:
                if status:
                    lots_num += 1 if lot.status == status else 0
                else:
                    lots_num += 1
        return lots_num

    
class OutputState:
    def __init__(self, state_model: Union[OutputStateModel, ModelProxy]):
        if isinstance(state_model, ModelProxy):
            self._model_proxy = state_model
        else:
            self._model_proxy = ModelProxy(state_model)

    @classmethod
    def from_dict(cls, state_dict: dict={}):
        model = OutputStateModel()
        model.location = state_dict['location']
        model.total_lot_capacity = proc_slots_num = state_dict['total_lot_capacity']
        model.lot_output_process = state_dict["lot_output_process"]
        model.lots_need_manual_removal = state_dict["lots_need_manual_removal"]
        slots = {str(slot_num): None for slot_num in range(proc_slots_num)}
        model.lot_slots = slots
        model.proc_slots = slots
        model.save()
        return cls(model)
    
    @property
    def location(self) -> Location:
        loc_dict = self._model_proxy.location
        return Location(node_id=loc_dict['node_id'],graph_id=loc_dict['graph_id'], frame_name='')
    
    @property
    def total_lot_capacity(self) -> int:
        return self._model_proxy.total_lot_capacity

    @property
    def lot_output_process(self) -> Dict:
        return self._model_proxy.lot_output_process
    
    @property
    def lots_need_manual_removal(self) -> bool:
        return self._model_proxy.lots_need_manual_removal
    
    @property
    def requested_robot_ops(self) -> List[Type[RobotOpDescriptor]]:
        return ListProxy(self._model_proxy.requested_robot_ops, RobotOpFactory.create_from_model)
    
    @property
    def lot_slots(self) -> Dict[str, Lot]:
        # to handle empty slots with None value
        modified_constructor = lambda model: Lot.from_object_id(model.object_id) if model else None
        return DictProxy(self._model_proxy.lot_slots, modified_constructor)

    @property
    def proc_slots(self) -> Dict[str, Type[StationProcess]]:
        # to handle empty slots with None value
        modified_constructor = lambda model: ProcessFactory.create_from_object_id(model.object_id) if model else None
        return DictProxy(self._model_proxy.proc_slots, modified_constructor)

    @property
    def procs_history(self) -> List[Type[StationProcess]]:
        return ListProxy(self._model_proxy.procs_history, ProcessFactory.create_from_model)

    def get_lots_num(self, status: LotStatus=None) -> int:
        lots_num = 0
        for _, lot in self.lot_slots.items():
            if lot is not None:
                if status:
                    lots_num += 1 if lot.status == status else 0
                else:
                    lots_num += 1
        return lots_num
    
    
    