from archemist.state.robot import Robot, RobotState
from archemist.state.station import Station, StationState, StationOpDescriptor
from archemist.persistence.object_factory import StationFactory
from typing import Tuple,Dict


class StationHandler:
    def __init__(self, station: Station):
        self._station = station
        self._station_sm = StationFactory.create_state_machine(self._station)

    def execute_op(self):
        pass

    def is_op_execution_complete(self) -> bool:
        pass

    def get_op_result(self) -> Tuple[bool,Dict]:
        pass

    def handle(self):
        self._station_sm.process_state_transitions()
        if self._station.state == StationState.OP_ASSIGNED:
            self._station.start_executing_op()
            self.execute_op()
        elif self._station.state == StationState.EXECUTING_OP:
            if self.is_op_execution_complete():
                op_successful, op_results = self.get_op_result()
                self._station.complete_assigned_station_op(op_successful, **op_results)
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

    def run(self):
        pass