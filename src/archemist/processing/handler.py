from archemist.state.robot import Robot, RobotState
from archemist.state.station import Station, StationState, StationOpDescriptor
from archemist.persistence.object_factory import StationFactory


class StationHandler:
    def __init__(self, station: Station):
        self._station = station
        self._station_sm = StationFactory.create_state_machine(self._station)

    def process(self):
        pass

    def handle(self):
        self._station_sm.process_state_transitions()
        if (self._station.state == StationState.WAITING_ON_OPERATION):
            station_op = self.process()
            if station_op is not None and station_op.has_result:
                self._station.complete_assigned_station_op(station_op.was_successful)
        
    def update_station_batch(self, operation_op):
        self._station.assign_station_op(operation_op)
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
        pass


    def handle(self):
        if self._robot.state == RobotState.JOB_ASSIGNED:
            self.execute_job()
        elif self._robot.state == RobotState.EXECUTING_JOB:
            if self.is_job_execution_complete():
                self._robot.complete_assigned_op(True) #TODO fix this to check that job execution was successful
                self._handled_robot_op = None

    def run(self):
        pass