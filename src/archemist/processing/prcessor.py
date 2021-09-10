from archemist.state.state import State
from archemist.persistence.persistenceManager import persistenceManager
from archemist.state.station import Station
from archemist.state.robot import Robot
import rospy
import roslaunch

class WorkflowManager:
    def __init__(self):
        self._persistanceManager = persistenceManager()
        self._state = None
        self._completed_batch_queue = []
        self._processing_batch_queue = []
        self._robot_queue = []

    def initializeWorkflow(self):
        self._state = State()
        self._state.initializeState()
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        for station in self._state.stations:
            stationHandlerName = station.__class__.__name__ + "_Handler"
            try:
                node = roslaunch.core.Node('archemist', stationHandlerName)
                process = launch.launch(node)
            except:
                print("Couldn't launch node: " + stationHandlerName)


    def process(self):
        while (True):
            self._state = self._persistanceManager.pull()
            for station in state.stations:
                self._checkStation(station)
            for robot in state.robots:
                self._checksRobot(robot)

            # rack processing
            self._processCompleteBatches()

            # vial processing
            self._processIncompleteBatches()



    def _checkStation(self, station: Station):
        if station.has_finished_batch():
            self._completed_batch_queue.append(station.retrieve_batch())
        elif station.has_processing_batch():
            self._processing_batch_queue.append(station.batch)

    def _checksRobot(self, robot: Robot):
        if robot.has_finished_batch():
            self._completed_batch_queue.append(robot.retrieve_batch())

    def _processCompleteBatches(self):
        while self._completed_batch_queue:
            batch = self._completed_batch_queue.pop()
            batch_current_station = batch.getLastStationStamp[0]
            batch_next_station = batch.getCurrentFlowNode().station
            if (batch_current_station != batch_next_station):
                self._robot_queue.append(batch)
            elif (batch_current_station == batch_next_station):
                self._advanceBatch(batch)

    def _advanceBatch(self, batch):
        for station in state.stations:
            if (station.__class__.__name__ == batch.getCurrentFlowNode().station):
                station.add_batch(batch)
                # in the station handler you start processing added batch and execute statioOp
