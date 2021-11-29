from archemist.exceptions.exception import Error
from archemist.state.state import State
from archemist.persistence.persistenceManager import persistenceManager, Parser
from archemist.state.station import Station, StationState
from archemist.state.robot import Robot, RobotState, VialMoveOpDescriptor, TransportBatchOpDescriptor, RackPlaceOpDescriptor
from archemist.state.batch import Batch
from archemist.state.robots.pandaFranka import PandaFranka, PandaMoveOpDescriptor
from archemist.state.robots.kukaLBRIIWA import KukaLBRIIWA, KukaMoveBaseOpDescriptor, KukaPickOpDescriptor, KukaRackPlaceOpDescriptor,KukaVialMoveOpDescriptor, RackPickOpDescriptor
from archemist.util.location import Location
from archemist.util.sm import States
from archemist.processing.scheduler import SimpleRobotScheduler
import rospy
import roslaunch
from time import sleep


class WorkflowManager:
    def __init__(self):
        self._persistanceManager = persistenceManager()
        self._state = None
        self._scheduler = SimpleRobotScheduler()
        
        self._job_station_queue = []
        self._unassigned_batches = []
        self._completed_batches = []

    def initializeWorkflow(self):
        self._state = State()
        self._state.initializeState(reset_db=True)
        kuka1 = self._state.getRobot('KukaLBRIIWA',1) 
        kuka1.location = Location(5,7,'drive_frame')
        self._state.modifyObjectDB(kuka1)
        panda = self._state.getRobot('PandaFranka',1)
        panda.location = Location(8,7,'neutral')
        self._state.modifyObjectDB(panda)
        # launch = roslaunch.scriptapi.ROSLaunch()
        # launch.start()
        # for station in self._state.stations:
        #     stationHandlerName = station.__class__.__name__ + "_Handler"
        #     try:
        #         node = roslaunch.core.Node('archemist', stationHandlerName)
        #         process = launch.launch(node)
        #     except:
        #         print("Couldn't launch node: " + stationHandlerName)

    def createBatch(self, batch_id, rows, cols, location):
        parser = Parser()
        self._unassigned_batches.append(Batch(batch_id, parser.loadRecipeYaml(), rows, cols, location))

    # this can keep track of the batches on the robot deck
    def process(self):
        while (True):
            self._state.updateFromDB()
            # process unassigned batches
            while self._unassigned_batches:
                batch = self._unassigned_batches.pop()
                if batch.recipe.is_complete():
                    self._completed_batches.append(batch)
                else:
                    current_station_name = batch.recipe.get_current_recipe_state().station
                    current_station = self._state.getStation(current_station_name)
                    if current_station.state == StationState.IDLE:
                        current_station.add_batch(batch)
                        self._state.modifyObjectDB(current_station)
                    else:
                        # TODO un assigned batch add back to the list
                        pass


            # process workflow stations
            for station in self._state.stations:
                if station.state == StationState.PROCESSING and station.has_robot_job():
                    robot_job = station.get_robot_job()
                    job_station_tuple = tuple((robot_job, station.__class__.__name__))
                    self._job_station_queue.append(job_station_tuple)
                    self._state.modifyObjectDB(station)
                elif station.has_processed_batch():
                    processed_batch = station.get_processed_batch()
                    processed_batch.recipe.advance_recipe_state(True)
                    self._unassigned_batches.append(processed_batch)
                    self._state.modifyObjectDB(station)

            # process workflow robots
            for robot in self._state.robots:
                if robot.has_complete_job():
                    (robot_job, station_name) = robot.get_complete_job()
                    # todo check the robot job matches
                    station_to_notify = self._state.getStation(station_name)
                    station_to_notify.finish_robot_job()
                    self._state.modifyObjectDB(station_to_notify)
                    self._state.modifyObjectDB(robot)

            self._scheduler.schedule(self._job_station_queue, self._state)
            sleep(1)


