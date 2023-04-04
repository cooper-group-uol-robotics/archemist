from archemist.core.state.robot import Robot, RobotState
from archemist.core.state.station import Station, StationState
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.state.station_process import StationProcessData
from typing import Tuple,Dict


class StationHandler:
    class ProcessHandler:
        def __init__(self, station: Station) -> None:
            self._station = station
            self._running_process = [StationFactory.create_station_process(self._station, process_data) 
                                     for process_data in self._station.get_all_processes_data()]

        def _process_assigned_batches(self):
            assigned_batches = self._station.assigned_batches
            new_batches = [batch for batch in assigned_batches + self._in_process_batches 
                           if batch in assigned_batches and batch not in self._in_process_batches]
            if len(new_batches) == self._station.process_batch_capacity:
                process_data = StationProcessData.from_args(new_batches)
                process = StationFactory.create_station_process(self._station, process_data)
                self._running_process.append(process)


        def _handle_processes(self):
            self._in_process_batches = []
            for process in list(self._running_process):
                if process.data.status['state'] != 'final_state':
                    process.process_state_transitions()
                    self._in_process_batches.extend(process.data.batches)
                else:
                    self._running_process.remove(process)
                    self._station.delete_process_data(process.data.uuid)

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
        if self._station.state == StationState.OP_ASSIGNED:
            self._station.start_executing_op()
            self.execute_op()
        elif self._station.state == StationState.EXECUTING_OP:
            if self.is_op_execution_complete():
                op_successful, op_results = self.get_op_result()
                self._station.complete_assigned_station_op(op_successful, **op_results)
                self._station.set_to_processing()
        elif self._station.state == StationState.REPEAT_OP:
            self._station.start_executing_op()
            self.execute_op()
        elif self._station.state == StationState.SKIP_OP:
            self._station.complete_assigned_station_op(True, **{})
            self._station.set_to_processing()

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