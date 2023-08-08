from archemist.core.state.robot import Robot, RobotState
from archemist.core.state.station import Station, OpState
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.state.station_process import StationProcessData
from typing import Tuple,Dict


class StationHandler:
    class ProcessHandler:
        def __init__(self, station: Station) -> None:
            self._station = station
            num_slots = int(station.batch_capacity/station.process_batch_capacity)
            self._processing_slots = {slot: None for slot in range(num_slots)}
            for process_data in self._station.get_all_processes_data():
                self._processing_slots[process_data.processing_slot] = StationFactory.create_station_process(self._station, process_data)
            

        def _process_assigned_batches(self):
            assigned_batches = self._station.assigned_batches
            new_batches = [batch for batch in assigned_batches + self._in_process_batches 
                           if batch in assigned_batches and batch not in self._in_process_batches]
            batches_per_process = self._station.process_batch_capacity
            if len(new_batches) > 0 and (len(new_batches) % batches_per_process) == 0:
                processes_split_batches_ = [new_batches[i: i + batches_per_process] for i in range(0, len(new_batches) , batches_per_process)]
                for batches in processes_split_batches_:
                    for slot, process in self._processing_slots.items():
                        if process is None:
                            process_data = StationProcessData.from_args(batches, slot)
                            new_process = StationFactory.create_station_process(self._station, process_data) #TODO batch or op process can be passed as an arg
                            self._processing_slots[slot] = new_process
                            break


        def _handle_processes(self):
            self._in_process_batches = []
            for slot, process in self._processing_slots.items():
                if process is not None:
                    if process.data.status['state'] != 'final_state':
                        process.process_state_transitions()
                        self._in_process_batches.extend(process.data.batches)
                    else:
                        if self._station.__class__.__name__ == "InputStation" or self._station.__class__.__name__ == "OutputStation":
                            if self._station.batches_manaully_removed:
                                self._station.delete_process_data(process.data.uuid)
                                self._processing_slots[slot] = None
                        else:
                            self._station.delete_process_data(process.data.uuid)
                            self._processing_slots[slot] = None
            if self._station.batches_manaully_removed:
                self._station.batches_manaully_removed = False

        def run_processes(self):
            self._handle_processes()
            self._process_assigned_batches()
    
    def __init__(self, station: Station):
        self._station = station
        self._process_handler = self.ProcessHandler(station)

    def execute_op(self):
        pass

    def is_op_execution_complete(self) -> bool:
        pass

    def get_op_result(self) -> Tuple[bool,Dict]:
        pass

    def handle(self):
        self._process_handler.run_processes()
        self._station.update_assigned_op()
        if self._station.assigned_op_state == OpState.ASSIGNED:
            self._station.add_timestamp_to_assigned_op()
            self._station.assigned_op_state = OpState.EXECUTING
            self.execute_op()
        elif self._station.assigned_op_state == OpState.EXECUTING:
            if self.is_op_execution_complete():
                op_successful, op_results = self.get_op_result()
                self._station.complete_assigned_station_op(op_successful, **op_results)
        elif self._station.assigned_op_state == OpState.TO_BE_REPEATED:
            self._station.assigned_op_state = OpState.ASSIGNED
        elif self._station.assigned_op_state == OpState.TO_BE_SKIPPED:
            self._station.complete_assigned_station_op(True, **{})

    def run(self):
        pass

class RobotHandler:
    def __init__(self, robot: Robot):
        self._robot = robot

    def execute_op(self):
        pass

    def is_op_execution_complete(self) -> bool:
        pass

    def get_op_result(self) -> bool:
        pass


    def handle(self):
        if self._robot.state == RobotState.OP_ASSIGNED:
            self._robot.start_executing_op()
            self.execute_op()
        elif self._robot.state == RobotState.EXECUTING_OP:
            if self.is_op_execution_complete():
                op_successful = self.get_op_result()
                self._robot.complete_assigned_op(op_successful)
                self._robot.set_to_execution_complete()
        elif self._robot.state == RobotState.REPEAT_OP:
            self._robot.start_executing_op()
            self.execute_op()
        elif self._robot.state == RobotState.SKIP_OP:
            self._robot.complete_assigned_op(True) #TODO might need to pass that operation was skipped
            self._robot.set_to_execution_complete()

    def run(self):
        pass