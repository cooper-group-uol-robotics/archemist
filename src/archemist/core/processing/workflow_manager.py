from archemist.core.persistence.yaml_handler import YamlHandler
from archemist.core.state.state import InputState, WorkflowState, OutputState
from archemist.core.processing.scheduler import RobotScheduler, RobotOpDescriptor
from archemist.core.processing.processor import InputProcessor, WorkflowProcessor, OutputProcessor
from archemist.core.persistence.recipe_files_watchdog import RecipeFilesWatchdog
from threading import Thread
from pathlib import Path
from time import sleep
from typing import Type

PROCESSING_SLEEP_DURATION = 1


class WorkflowManager:
    def __init__(self, input_state: InputState,
                 workflow_state: WorkflowState,
                 output_state: OutputState,
                 robot_scheduler: RobotScheduler,
                 recipes_dir: Path):
        
        self._input_processor = InputProcessor(input_state)
        self._workflow_processor = WorkflowProcessor(workflow_state)
        self._output_processor = OutputProcessor(output_state)
        self._robot_scheduler = robot_scheduler
        self._recipes_watchdog = RecipeFilesWatchdog(recipes_dir)

        self._processing_thread = None
        self._running = False
        self._pause_workflow = False

    def start_workflow(self):
        self._running = True
        self._processor_thread = Thread(target=self._process, daemon=True)
        self._recipes_watchdog.start()
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

    def add_clean_batch(self):
        return self._input_processor.add_clean_batch()

    def remove_lot(self, slot: int):
        return self._output_processor.remove_lot(str(slot))

    def remove_all_lots(self):
        return self._output_processor.remove_all_lots()
    
    def queue_robot_op(self, robot_op: Type[RobotOpDescriptor]):
        self._workflow_processor.robot_ops_queue.append(robot_op)

    def _process(self):
        while self._running:
            if not self._pause_workflow:
                self._queue_added_recipes()
                
                # input processor update 
                self._input_processor.process_lots()
                
                # add input processor requested ops to the robot ops queue
                while self._input_processor.requested_robot_ops:
                    robot_op = self._input_processor.requested_robot_ops.pop(left=True)
                    self._workflow_processor.robot_ops_queue.append(robot_op)

                # workflow processor update
                new_lots = self._input_processor.retrieve_ready_for_collection_lots()
                if new_lots:
                    self._workflow_processor.add_ready_for_collection_lots(new_lots)
                
                self._workflow_processor.process_workflow()

                # output processor update
                if self._output_processor.has_free_lot_capacity():
                    complete_lot = self._workflow_processor.retrieve_completed_lot()
                    if complete_lot:
                        self._output_processor.add_lot(complete_lot)

                self._output_processor.process_lots()

                # add output processor requested ops to the robot ops queue
                while self._output_processor.requested_robot_ops:
                    robot_op = self._output_processor.requested_robot_ops.pop(left=True)
                    self._workflow_processor.robot_ops_queue.append(robot_op)

            # schedule robot ops
            if self._workflow_processor.robot_ops_queue:
                self._robot_scheduler.schedule(self._workflow_processor.robot_ops_queue)

            sleep(PROCESSING_SLEEP_DURATION)

    def _queue_added_recipes(self):
        while self._recipes_watchdog.recipes_queue:
                recipe_file_path = self._recipes_watchdog.recipes_queue.popleft()
                recipe_dict = YamlHandler.load_recipe_file(recipe_file_path)
                self._input_processor.add_recipe(recipe_dict)

    def _log_processor(self, message:str):
        print(f'[{self.__class__.__name__}]: {message}')