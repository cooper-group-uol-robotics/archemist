from archemist.state.robot import Robot, RobotState
from archemist.persistence.objectConstructor import ObjectConstructor
from archemist.state.station import Station, StationState, StationOpDescriptor


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
            if station_op is not None:
                self._station.finish_station_op(station_op)
        
    def update_station_batch(self, operation_op):
        self._station.set_station_op(operation_op)
        if (self._station_sm.batch_mode):
            for _ in range(0, self._station.assigned_batch.num_samples):
                self._station.assigned_batch.add_station_op_to_current_sample(operation_op)
                self._station.assigned_batch.process_current_sample()
        else:
            self._station.assigned_batch.add_station_op_to_current_sample(operation_op)
            self._station.assigned_batch.process_current_sample()

    def run(self):
        pass

class RobotHandler:
    def __init__(self, robot: Robot):
        self._robot = robot
        self._handled_robot_op = None

    def execute_job(self):
        pass

    def is_job_execution_complete(self):
        return False


    def handle(self):
        if self._robot.state == RobotState.JOB_ASSIGNED:
            self.execute_job()
        elif self._robot.state == RobotState.EXECUTING_JOB:
            if self.is_job_execution_complete():
                self._robot.complete_assigned_job(self._handled_robot_op)
                self._handled_robot_op = None

    def run(self):
        pass