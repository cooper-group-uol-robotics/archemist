from archemist.state.robot import Robot, RobotState
from archemist.state.state import State
from archemist.persistence.objectConstructor import ObjectConstructor
from archemist.state.station import Station, StationState


class StationHandler:
    def __init__(self, station: Station):
        self._station = station
        self._station_sm = ObjectConstructor.construct_process_sm_for_station(self._station)

    def process(self):
        pass

    def handle(self):
        self._station_sm.process_state_transitions()
        if (self._station.state == StationState.WAITING_ON_OPERATION):
            station_op = self.process()
            self.update_station_batch(station_op)
            self._station.finish_station_operation()
        
    def update_station_batch(self, operation_op):
        self._station.set_station_op(operation_op)
        if (self._station_sm.batch_mode):
            for _ in range(0, self._station.assigned_batch.num_samples):
                self._station.assigned_batch.add_station_op_to_current_sample(operation_op)
                self._station.assigned_batch.process_current_sample()
        else:
            self._station.assigned_batch.add_station_op_to_current_sample(operation_op)
            self._station.assigned_batch.process_current_sample()

class RobotHandler:
    def __init__(self, robot: Robot):
        self._robot = robot

    def execute_job(self):
        pass


    def handle(self):
        if (self._robot.state == RobotState.EXECUTING_JOB):
            station_robot_job = self.execute_job()
            self._robot.complete_assigned_job(station_robot_job)