from archemist.core.state.state import State
from archemist.core.state.station import StationState
from archemist.core.state.batch import Batch
from archemist.core.util.location import Location
from archemist.core.processing.scheduler import MultiBatchRobotScheduler
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

    def is_running(self) -> bool:
        return self._running

    def start_processor(self):
        self._running = True
        self._processor_thread = Thread(target=self._process, daemon=True)
        self._processor_thread.start()
        self._log_processor('processor thread is started')

    def stop_processor(self):
        self._running = False
        self._processor_thread.join(1)
        self._log_processor('processor thread is terminated')

    def pause_processor(self):
        self._pause_workflow = True

    def resume_processor(self):
        self._pause_workflow = False

    def queue_recipe(self, recipe_dict):
        self._workflow_state.queue_recipe(recipe_dict)

    def queue_robot_op(self, robot_op):
        self._workflow_state.robot_ops_queue.append(robot_op)


    # this can keep track of the batches on the robot deck
    def _process(self):
        while (self._running):
            if not self._pause_workflow:
                # process queued recipes
                if self._workflow_state.recipes_queue:
                    clean_batches = self._workflow_state.get_clean_batches()
                    while self._workflow_state.recipes_queue:
                        if clean_batches: 
                            recipe = self._workflow_state.recipes_queue.popleft()
                            new_batch = clean_batches.popleft()
                            new_batch.attach_recipe(recipe)
                            new_batch.recipe.advance_state(True) # to move out of start state
                            self._workflow_state.batches_buffer.append(new_batch)
                        else:
                            break

                # process unassigned batches
                batches_buffer_copy = [batch for batch in self._workflow_state.batches_buffer]
                for batch in batches_buffer_copy: # here the list gets copied
                    if batch.recipe.is_complete():
                        self._log_processor(f'{batch} recipe is complete. The batch is added to the complete batches list')
                        self._workflow_state.batches_buffer.remove(batch)
                    else:
                        station_name, station_id = batch.recipe.get_current_station()
                        current_station = self._workflow_state.get_station(station_name, station_id)
                        self._log_processor(f'Trying to assign ({batch}) to {current_station}')
                        if current_station.state == StationState.IDLE and current_station.has_free_batch_capacity():
                            current_station.add_batch(batch)
                            self._workflow_state.batches_buffer.remove(batch)
                        else:
                            self._log_processor(f'{batch} could not be assigned to {current_station}.')


                # process workflow stations
                for station in self._workflow_state.stations:
                    if station.state == StationState.PROCESSING and station.has_requested_robot_op():
                        robot_job = station.get_requested_robot_op()
                        self._log_processor(f'{robot_job} is added to robot scheduling queue.')
                        self._workflow_state.robot_ops_queue.append(robot_job)
                    elif station.has_processed_batch():
                        processed_batch = station.get_processed_batch()
                        processed_batch.recipe.advance_state(True)
                        self._log_processor(f'Processing {processed_batch} is complete. Adding to the unassigned list.')
                        self._workflow_state.batches_buffer.append(processed_batch)

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

            if self._workflow_state.robot_ops_queue:
                self._robot_scheduler.schedule(self._workflow_state)
            
            sleep(2)

    def _log_processor(self, message:str):
        print(f'[{self}]: {message}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}'


