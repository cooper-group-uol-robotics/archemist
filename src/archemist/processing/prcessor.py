from archemist.state.state import State
from archemist.state.station import StationState
from archemist.state.batch import Batch
from archemist.persistence.yamlHandler import YamlHandler
from archemist.util.location import Location
from archemist.processing.scheduler import SimpleRobotScheduler
from collections import deque
from threading import Thread
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
        self._processor_thread = Thread(target=self._process, daemon=True)
        self._processor_thread.start()
        self._log_processor('processor thread is started')

    def stop_processor(self):
        self._running = False
        self._processor_thread.join(1)
        self._log_processor('processor thread is terminated')

    def add_batch(self, batch_id, recipe_file_path, num_samples, location):
        recipe_dict = YamlHandler.loadYamlFile(recipe_file_path)
        new_batch = Batch.from_arguments(self._workflow_state.db_name, batch_id, recipe_dict, 
                                        num_samples, location)
        new_batch.recipe.advance_state(True) # to move out of start state
        self._unassigned_batches.append(new_batch)

    # this can keep track of the batches on the robot deck
    def _process(self):
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
                    processed_batch.recipe.advance_state(True)
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
                    station_to_notify.finish_robot_job(station_robot_job.robot_op)

            if self._job_station_queue:
                self._robot_scheduler.schedule(self._job_station_queue, self._workflow_state)
            
            sleep(5)

    def _log_processor(self, message:str):
        print(f'[{self}]: {message}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}'


