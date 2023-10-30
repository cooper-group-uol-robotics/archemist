from archemist.core.persistence.objects_getter import StationsGetter, RecipesGetter
from archemist.core.persistence.object_factory import ProcessFactory
from archemist.core.state.state import InputState, WorkflowState, OutputState
from archemist.core.state.batch import Batch
from archemist.core.state.recipe import Recipe
from archemist.core.state.robot_op import RobotOpDescriptor
from archemist.core.state.lot import Lot, LotStatus
from archemist.core.util.enums import ProcessStatus
from typing import List, Type, Dict

class InputProcessor:

    def __init__(self, input_state: InputState):
        self._state = input_state

    @property
    def requested_robot_ops(self) -> List[Type[RobotOpDescriptor]]:
        return self._state.requested_robot_ops

    def add_clean_batch(self):
        current_num_lots = self._state.get_lots_num()
        total_batches_num = len(self._state.batches_queue) + current_num_lots*self._state.batches_per_lot
        if total_batches_num < self._state.total_lot_capacity*self._state.batches_per_lot:
            new_batch = Batch.from_args(self._state.samples_per_batch, self._state.location)
            self._state.batches_queue.append(new_batch)
        else:
            self._log("maximum number of batches is reached. Cannot add a new batch")

    def add_recipe(self, recipe_dict: Dict):
        new_recipe_id = recipe_dict["general"]["id"]
        if not RecipesGetter.recipe_exists(new_recipe_id):
            self._state.recipes_queue.append(Recipe.from_dict(recipe_dict))
            self._log(f"new recipe with id {new_recipe_id} queued")
        else:
            self._log(f"recipe with id {new_recipe_id} already exists. Recipe is not added to queue")

    def process_lots(self):
        for slot, lot in self._state.lot_slots.items():
            if lot is None:
                batches_per_lot = self._state.batches_per_lot
                if len(self._state.batches_queue) >= batches_per_lot:
                    new_lot_batches = []
                    for _ in range(batches_per_lot):
                        new_lot_batches.append(self._state.batches_queue.pop(left=True))
                    new_lot = Lot.from_args(new_lot_batches)
                    self._state.lot_slots[slot] = new_lot
                    self._log(f"{new_lot} is created")
            else:
                if lot.status == LotStatus.CREATED:
                    if self._state.recipes_queue:
                        new_recipe = self._state.recipes_queue.pop(left=True)
                        lot.attach_recipe(new_recipe)
                        lot.status = LotStatus.STANDBY
                elif lot.status == LotStatus.STANDBY:
                    # check if next station has capacity
                    next_step_details = lot.recipe.current_state_details
                    next_station = StationsGetter.get_station(next_step_details.station_id, next_step_details.station_type)
                    num_prepared_lots = self._state.get_lots_num(LotStatus.ONBOARDING) + self._state.get_lots_num(LotStatus.READY_FOR_COLLECTION)
                    if num_prepared_lots < next_station.free_lot_capacity:
                        if self._state.lot_input_process is not None:
                            new_proc_dict = self._state.lot_input_process
                        else:
                            new_proc_dict = {"type": "StationProcess", "args": None}
                        
                        new_proc = ProcessFactory.create_from_dict(new_proc_dict, lot)
                        self._state.proc_slots[slot] = new_proc
                        new_proc.lot_slot = slot
                        lot.status = LotStatus.ONBOARDING

                elif lot.status == LotStatus.ONBOARDING:
                    proc = self._state.proc_slots[slot]
                    if proc.m_state != "final_state":
                        proc.tick()
                        if proc.status == ProcessStatus.REQUESTING_ROBOT_OPS:
                            for robot_op in proc.req_robot_ops:
                                robot_op.requested_by = self._state.object_id
                                self._state.requested_robot_ops.append(robot_op)
                            proc.switch_to_waiting()
                    else:
                        self._state.proc_slots[slot] = None
                        self._state.procs_history.append(proc)
                        lot.status = LotStatus.READY_FOR_COLLECTION
                        self._log(f"{lot} is ready for collection")
                        
    def retrieve_ready_for_collection_lots(self) -> List[Lot]:
        ready_for_collection_lots = []
        for slot, lot in self._state.lot_slots.items():
            if lot is not None and lot.status == LotStatus.READY_FOR_COLLECTION:
                ready_for_collection_lots.append(lot)
                self._state.lot_slots[slot] = None
        
        return ready_for_collection_lots
    
    def _log(self, message:str):
        print(f'[{self.__class__.__name__}]: {message}')
    
class OutputProcessor:
    def __init__(self, output_state: OutputState):
        self._state = output_state

    @property
    def requested_robot_ops(self) -> List[Type[RobotOpDescriptor]]:
        return self._state.requested_robot_ops

    def has_free_lot_capacity(self) -> bool:
        return self._state.get_lots_num() < self._state.total_lot_capacity

    def add_lot(self, added_lot: Lot):
        for slot, lot in self._state.lot_slots.items():
            if lot is None:
                self._state.lot_slots[slot] = added_lot
                self._log(f"{added_lot} is added to output slot {slot}")
                break

    def process_lots(self):
        for slot, lot in self._state.lot_slots.items():
            if lot:
                if lot.status == LotStatus.IN_WORKFLOW:
                    if self._state.lot_output_process is not None:
                        new_proc_dict = self._state.lot_output_process
                    else:
                        new_proc_dict = {"type": "StationProcess", "args": None}

                    new_proc = ProcessFactory.create_from_dict(new_proc_dict, lot)
                    self._state.proc_slots[slot] = new_proc
                    new_proc.lot_slot = slot
                    lot.status = LotStatus.OFFBOARDING
                
                elif lot.status == LotStatus.OFFBOARDING:
                    proc = self._state.proc_slots[slot]
                    if proc.m_state != "final_state":
                        proc.tick()
                        if proc.status == ProcessStatus.REQUESTING_ROBOT_OPS:
                            for robot_op in proc.req_robot_ops:
                                robot_op.requested_by = self._state.object_id
                                self._state.requested_robot_ops.append(robot_op)
                            proc.switch_to_waiting()
                    else:
                        self._state.proc_slots[slot] = None
                        self._state.procs_history.append(proc)
                        if self._state.lots_need_manual_removal:
                            lot.status = LotStatus.NEED_REMOVAL
                            self._log(f"{lot} is waiting for manual removal")
                        else:
                            lot.status = LotStatus.FINISHED
                            self._state.lot_slots[slot] = None
                            self._log(f"{lot} processing is finished")

    def remove_lot(self, slot: str):
        lot = self._state.lot_slots[slot]
        if lot and lot.status == LotStatus.NEED_REMOVAL:
            lot.status = LotStatus.FINISHED
            self._state.lot_slots[slot] = None
            self._log(f"{lot} was manually removed. Lot processing is finished")
        else:
            self._log(f"unable to remove lot at output slot {slot}")

    def remove_all_lots(self):
        for slot, lot in self._state.lot_slots.items():
            if lot and lot.status == LotStatus.NEED_REMOVAL:
                self.remove_lot(slot)

    def _log(self, message:str):
        print(f'[{self.__class__.__name__}]: {message}')

class WorkflowProcessor:
    def __init__(self, workflow_state: WorkflowState):
        self._state = workflow_state

    @property
    def robot_ops_queue(self) -> List[Type[RobotOpDescriptor]]:
        return self._state.robot_ops_queue

    def add_ready_for_collection_lots(self, collection_lots: List[Lot]):
        for lot in collection_lots:
            lot.status = LotStatus.IN_WORKFLOW
            self._state.lots_buffer.append(lot)

    def process_workflow(self):
        # process lots buffer
        lots_buffer = [lot for lot in self._state.lots_buffer]
        for lot in lots_buffer:
            if not lot.recipe.is_complete():
                recipe_state_details = lot.recipe.current_state_details
                current_station_type = recipe_state_details.station_type
                current_station_id = recipe_state_details.station_id
                current_station = StationsGetter.get_station(current_station_id, current_station_type)
                self._log_processor(f'Trying to assign ({lot}) to {current_station}')
                if current_station.free_lot_capacity > 0:
                    current_station.add_lot(lot)
                    self._state.lots_buffer.remove(lot)
                else:
                    self._log_processor(f'{lot} could not be assigned to {current_station}')
        
        # process workflow stations
        for station in StationsGetter.get_stations():
            # get requested robot ops
            while station.requested_robot_ops:
                robot_op = station.requested_robot_ops.pop(left=True)
                self._log_processor(f'{robot_op} is added to robots scheduling queue')
                self._state.robot_ops_queue.append(robot_op)

            # get requested external station procs
            while station.requested_ext_procs:
                ext_proc = station.requested_ext_procs.pop(left=True)
                self._log_processor(f'{ext_proc} is added to processes queue')
                self._state.proc_requests_queue.append(ext_proc)

            # get processed lots
            ready_for_collection_lots = station.retrieve_ready_for_collection_lots()
            for lot in ready_for_collection_lots:
                lot.recipe.advance_state(success=True)
                self._log_processor(f'Processing {lot} is complete. Adding to lots buffer')
                lot.status = LotStatus.IN_WORKFLOW
                self._state.lots_buffer.append(lot)

        # process external processes requests
        while self._state.proc_requests_queue:
            ext_proc = self._state.proc_requests_queue.pop()
            req_station_type = ext_proc.associated_station
            station = StationsGetter.get_station(req_station_type)
            station.add_process(ext_proc)

    def retrieve_completed_lot(self) -> Lot:
        completed_lot = None
        lots_buffer = [lot for lot in self._state.lots_buffer]
        for lot in lots_buffer:
            if lot.recipe.is_complete():
                completed_lot = lot
                self._state.lots_buffer.remove(lot)
                break
        return completed_lot

    def _log_processor(self, message:str):
        print(f'[{self}]: {message}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}'
    