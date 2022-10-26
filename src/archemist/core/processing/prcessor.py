from archemist.core.state.state import State
from archemist.core.state.station import StationState
from archemist.core.state.batch import Batch
from archemist.core.util.location import Location
from archemist.core.util.station_robot_job import StationRobotJob
from archemist.core.processing.scheduler import SimpleRobotScheduler,MultiBatchRobotScheduler
from collections import deque
from threading import Thread
from time import sleep


class WorkflowManager:
    def __init__(self, workflow_state: State):
        self._workflow_state = workflow_state
        self._robot_scheduler = MultiBatchRobotScheduler()
        self._processor_thread = None
        self._running = False
        self._pause_workflow = False
        self._current_batch_processing_count = 0
        self._max_batch_processing_capacity = 2
        
        self._job_station_queue = list()
        self._queued_recipes = deque()
        self._unassigned_batches = deque()

    @property
    def pause_workflow(self):
        return self._pause_workflow

    @pause_workflow.setter
    def pause_workflow(self, value):
        if isinstance(value, bool):
            self._pause_workflow = value
        else:
            raise ValueError

    @property
    def recipes_queue(self):
        return self._queued_recipes

    def start_processor(self):
        self._running = True
        self._processor_thread = Thread(target=self._process, daemon=True)
        self._processor_thread.start()
        self._log_processor('processor thread is started')

    def stop_processor(self):
        self._running = False
        self._processor_thread.join(1)
        self._log_processor('processor thread is terminated')

    def queue_recipe(self, recipe_dict):
        self._queued_recipes.append(recipe_dict)

    def queue_robot_op(self, robot_op):
        queued_job = StationRobotJob(robot_op=robot_op, station_obj_id=None)
        self._job_station_queue.append(queued_job)


    # this can keep track of the batches on the robot deck
    def _process(self):
        while (self._running):
            if not self._pause_workflow:
                # process queued recipes
                if self._queued_recipes:
                    clean_batches = self._workflow_state.get_clean_batches()
                    while self._queued_recipes:
                        if clean_batches and self._current_batch_processing_count < self._max_batch_processing_capacity: 
                            recipe = self._queued_recipes.pop()
                            new_batch = clean_batches.pop(0)
                            new_batch.attach_recipe(recipe)
                            self._current_batch_processing_count += 1
                            new_batch.recipe.advance_state(True) # to move out of start state
                            self._unassigned_batches.append(new_batch)
                        else:
                            break

                # process unassigned batches
                for batch in list(self._unassigned_batches): # here the list gets copied
                    if batch.recipe.is_complete():
                        self._log_processor(f'{batch} recipe is complete. The batch is added to the complete batches list')
                        self._current_batch_processing_count -= 1
                        self._unassigned_batches.remove(batch)
                    else:
                        station_name, station_id = batch.recipe.get_current_station()
                        current_station = self._workflow_state.get_station(station_name, station_id)
                        self._log_processor(f'Trying to assign ({batch}) to {current_station}')
                        if current_station.state == StationState.IDLE and current_station.has_free_batch_capacity():
                            current_station.add_batch(batch)
                            self._unassigned_batches.remove(batch)
                        else:
                            self._log_processor(f'{batch} could not be assigned to {current_station}.')


                # process workflow stations
                for station in self._workflow_state.stations:
                    if station.state == StationState.PROCESSING and station.has_requested_robot_op():
                        robot_job = station.get_requested_robot_op()
                        self._log_processor(f'{robot_job} is added to robot scheduling queue.')
                        self._job_station_queue.append(robot_job)
                    elif station.has_processed_batch():
                        processed_batch = station.get_processed_batch()
                        processed_batch.recipe.advance_state(True)
                        self._log_processor(f'Processing {processed_batch} is complete. Adding to the unassigned list.')
                        self._unassigned_batches.append(processed_batch)

            # process workflow robots
            for robot in self._workflow_state.robots:
                if robot.is_assigned_op_complete():
                    robot_job = robot.get_complete_op()
                    self._log_processor(f'{robot} finished executing job {robot_job}')
                    # todo check the robot job matches
                    if robot_job.origin_station is not None:
                        station_to_notify = self._workflow_state.get_station(robot_job.origin_station)
                        self._log_processor(f'Notifying {station_to_notify}')
                        station_to_notify.complete_robot_op_request(robot_job)

            if self._job_station_queue:
                self._robot_scheduler.schedule(self._job_station_queue, self._workflow_state)
            
            sleep(2)

    def _log_processor(self, message:str):
        print(f'[{self}]: {message}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}'


