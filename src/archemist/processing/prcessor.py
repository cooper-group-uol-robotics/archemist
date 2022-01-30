from archemist.state.state import State
from archemist.state.station import Station, StationState
from archemist.state.robot import Robot, RobotState
from archemist.state.batch import Batch
from archemist.state.robots.pandaFranka import PandaFranka
from archemist.state.robots.kukaLBRIIWA import KukaLBRIIWA
from archemist.util.location import Location
from archemist.processing.scheduler import SimpleRobotScheduler
from collections import deque
from threading import Thread
import rospy
import roslaunch
from time import sleep


class WorkflowManager:
    def __init__(self, workflow_state: State):
        self._workflow_state = workflow_state
        self._robot_scheduler = SimpleRobotScheduler()
        self._processor_thread = None
        self._running = False
        
        self._job_station_queue = list()
        self._unassigned_batches = deque()
        self._completed_batches = []

    def start_processor(self):
        self._running = True
        self._processor_thread = Thread(target=self.process, daemon=True)

    def initializeWorkflow(self):
        # kuka1 = self._state.getRobot('KukaLBRIIWA',1) 
        # kuka1.location = Location(5,7,'drive_frame')
        # self._state.modifyObjectDB(kuka1)
        panda = self._state.getRobot('PandaFranka',1)
        panda.location = Location(1,7,'neutral')
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

    def createBatch(self, batch_id, num_sample, location):
        self._unassigned_batches.append(Batch(batch_id, parser.loadRecipeYaml(), num_sample, location))

    # this can keep track of the batches on the robot deck
    def process(self):
        while (self._running):
            # process unassigned batches
            while self._unassigned_batches:
                batch = self._unassigned_batches.popleft()
                if batch.recipe.is_complete():
                    self._log_processor(f'{batch} recipe is complete. The batch is added to the complete batches list')
            
                else:
                    station_name, station_id = batch.recipe.get_current_station()
                    current_station = self._workflow_state.get_station(station_name, station_id)
                    self._log_processor(f'Trying to assign ({batch}) to {current_station}')
                    if current_station.state == StationState.IDLE:
                        current_station.add_batch(batch)
                    else:
                        self._log_processor(f'{batch} could not be assigned to {current_station}.')
                        # put unassigned batch back to the list since station is busy
                        self._unassigned_batches.append(batch)


            # process workflow stations
            for station in self._workflow_state.stations:
                if station.state == StationState.PROCESSING and station.has_robot_job():
                    #TODO change tuple to StationRobotJob
                    station_robot_job = station.get_robot_job()
                    self._log_processor(f'{station_robot_job.robot_op} is added to robot scheduling queue.')
                    self._job_station_queue.append(station_robot_job)
                elif station.has_processed_batch():
                    processed_batch = station.get_processed_batch()
                    processed_batch.recipe.advance_recipe_state(True)
                    self._log_processor(f'Processing {processed_batch} is complete. Adding to the unassigned list.')
                    self._unassigned_batches.append(processed_batch)

            # process workflow robots
            for robot in self._workflow_state.robots:
                if robot.has_complete_job():
                    station_robot_job = robot.get_complete_job()
                    self._log_processor(f'{robot} finished executing job {station_robot_job.robot_op}')
                    # todo check the robot job matches
                    station_to_notify = self._workflow_state.get_station(station_robot_job.station_obj_id)
                    self._log_processor(f'Notifying {station_to_notify}')
                    station_to_notify.finish_robot_job()

            if self._job_station_queue:
                self._robot_scheduler.schedule(self._job_station_queue, self._workflow_state)
            
            sleep(5)

    def _log_processor(self, message:str):
        print(f'[{self}]: {message}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}'


