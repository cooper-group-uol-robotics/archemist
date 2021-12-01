from archemist.state.robot import RobotState
from archemist.state.state import State
from archemist.persistence.persistenceManager import Parser
from archemist.state.station import StationState


class StationHandler:
    def __init__(self, station_name: str):
        self._state = State()
        self._state.initializeState(False)
        self._station_name = station_name
        self._station = self._state.getStation(station_name)
        
        parser = Parser()
        self._station_sm = parser.create_process_sm(self._station_name)
        self._station_sm.set_station(self._station)

    def process(self):
        pass

    def handle(self):
        self._state.updateFromDB()
        self._station = self._state.getStation(self._station_name)

        self._station_sm.set_station(self._station)
        self._station_sm.process_state_transitions()
        if (self._station.state == StationState.WAITING_ON_OPERATION):
            station_op = self.process()
            self.update_station_batch(station_op)
            self._station.state = StationState.OPERATION_COMPLETE
        
        self._state.modifyObjectDB(self._station)

    def update_station_batch(self, operation_op):
        self._station.set_station_op(operation_op)
        if (self._station_sm.batch_mode):
            for _ in range(0, self._station.assigned_batch.num_samples):
                self._station.assigned_batch.get_current_sample().add_operation_op(operation_op)
                self._station.assigned_batch.process_current_sample()
        else:
            self._station.assigned_batch.get_current_sample().add_operation_op(operation_op)
            self._station.assigned_batch.process_current_sample()

class RobotHandler:
    def __init__(self, robot_name: str, robot_id: int):
        self._state = State()
        self._state.initializeState(False)
        self._robot_name = robot_name
        self._robot_id = robot_id
        self._robot = self._state.getRobot(self._robot_name, self._robot_id)

    def execute_job(self):
        pass


    def handle(self):
        self._state.updateFromDB()
        self._robot = self._state.getRobot(self._robot_name, self._robot_id)

        if (self._robot.state == RobotState.EXECUTING_JOB):
            self.execute_job()
            self._robot.complete_assigned_job()
            self._state.modifyObjectDB(self._robot)