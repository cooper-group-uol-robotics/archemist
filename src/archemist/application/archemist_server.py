from time import sleep
from archemist.core.processing.prcessor import WorkflowManager
from archemist.core.persistence.persistence_manager import PersistenceManager
from archemist.core.persistence.recipe_files_watchdog import RecipeFilesWatchdog
from archemist.core.optimisation.optimisation_manager import OptimisationManager
from archemist.core.persistence.yaml_handler import YamlHandler
from archemist.core.util.location import Location
from pathlib import Path
from datetime import datetime
from archemist.robots.kmriiwa_robot.state import KukaLBRMaintenanceTask
from archemist.application.cmd_message import CMDCategory,CMDMessage
import zmq
import json

class ArchemistServer:
    def __init__(self, workflow_dir: Path, existing_db: bool=False, use_optimiser: bool=False) -> None:
        server_config_file_path = workflow_dir.joinpath(f'config_files/server_settings.yaml')
        self._server_config = YamlHandler.load_server_settings_file(server_config_file_path)
        
        workflow_config_file_path = workflow_dir.joinpath(f'config_files/workflow_config.yaml')
        recipes_dir_path = workflow_dir.joinpath(f'recipes')

        db_name = self._server_config['db_name']

        # Construct state from config file
        mongo_host= self._server_config['mongodb_host']
        persistence_mgr = PersistenceManager(mongo_host,db_name)
        if not existing_db:
            print('constructing new workflow state from config file')
            self._state = persistence_mgr.construct_state_from_config_file(workflow_config_file_path)
        else:
            print('reconstructing workflow state from existing database')
            self._state = persistence_mgr.construct_state_from_db()
        
        self._workflow_mgr = WorkflowManager(self._state)

        context = zmq.Context()
        self._server = context.socket(zmq.PAIR)
        self._server.bind('tcp://127.0.0.1:5555')
        
        self._recipes_watchdog = RecipeFilesWatchdog(recipes_dir_path)
        self._recipes_watchdog.start()
        print(f'[{datetime.now().strftime("%H:%M:%S")}] Archemist server started')

        if use_optimiser:
            self._optimiser_manager = OptimisationManager(workflow_dir, self._state, existing_db)
        else:
            self._optimiser_manager = None


    def run(self):
        # spin
        while True:
            self._queue_added_recipes()
            try:
                json_msg = self._server.recv_json(flags=zmq.NOBLOCK)
                msg = CMDMessage.from_json(json_msg)
                if msg.category == CMDCategory.WORKFLOW:
                    if msg.cmd == 'start':
                        if not self._workflow_mgr.is_running():
                            self._workflow_mgr.start_processor()
                        else:
                            self._workflow_mgr.resume_processor()
                    elif msg.cmd == 'pause':
                        self._workflow_mgr.pause_processor()
                    elif msg.cmd == 'add_batch':
                        self._state.add_clean_batch()
                    elif msg.cmd == 'start_optim':
                        if self._optimiser_manager:
                            self._optimiser_manager.start_optimisation()
                        else:
                            print("Optimisation manager not available. Cannot start optimisation")
                    elif msg.cmd == 'generate_recipe':
                        if self._optimiser_manager:
                            self._optimiser_manager.generate_new_recipe()
                        else:
                            print("Optimisation manager not available. Cannot generate new recipe")
                    elif msg.cmd == 'manual_batch_removal':
                        for station in self._state.stations:
                            if not station.batches_manaully_removed:
                                station.batches_manaully_removed = True
                    elif msg.cmd == 'terminate':
                        self.shut_down()
                        break
                elif msg.category == CMDCategory.ROBOT:
                    if msg.cmd == 'get_list':
                        robots = self._state.robots
                        robots_dict = {
                            'names': [robot.__class__.__name__ for robot in robots],
                            'ids':  [robot.id for robot in robots],
                        }
                        json_msg = json.dumps(robots_dict)
                        self._server.send_json(json_msg)
                    elif msg.cmd == 'repeat_op':
                        robot = self._state.get_robot(msg.params[0], msg.params[1])
                        robot.repeat_assigned_op()
                    elif msg.cmd == 'skip_op':
                        robot = self._state.get_robot(msg.params[0], msg.params[1])
                        robot.skip_assigned_op()
                    elif msg.cmd == 'charge':
                        #TODO make queue_robot_op accept specific robot 
                        self._workflow_mgr.queue_robot_op(KukaLBRMaintenanceTask.from_args('ChargeRobot',[False,85]))
                    elif msg.cmd == 'stop_charge':
                        self._workflow_mgr.queue_robot_op(KukaLBRMaintenanceTask.from_args('StopCharge',[False]))
                    elif msg.cmd == 'resume_app':
                        self._workflow_mgr.queue_robot_op(KukaLBRMaintenanceTask.from_args('resumeLBRApp',[False]))
                elif msg.category == CMDCategory.STATION:
                    if msg.cmd == 'get_list':
                        stations = self._state.stations
                        stations_dict = {
                            'names': [station.__class__.__name__ for station in stations],
                            'ids':  [station.id for station in stations],
                        }
                        json_msg = json.dumps(stations_dict)
                        self._server.send_json(json_msg)
                    elif msg.cmd == 'repeat_op':
                        station = self._state.get_station(msg.params[0], msg.params[1])
                        station.repeat_assigned_op()
                    elif msg.cmd == 'skip_op':
                        station = self._state.get_station(msg.params[0], msg.params[1])
                        station.skip_assigned_op()
            except zmq.ZMQError:
                sleep(0.2)
            except KeyboardInterrupt:
                self.shut_down()
                break

    def _queue_added_recipes(self):
        while self._recipes_watchdog.recipes_queue:
                recipe_file_path = self._recipes_watchdog.recipes_queue.popleft()
                recipe_dict = YamlHandler.load_recipe_file(recipe_file_path)
                self._workflow_mgr.queue_recipe(recipe_dict)
                print(f'new recipe with id {recipe_dict["general"]["id"]} queued')

    def shut_down(self):
        if self._workflow_mgr.is_running():
            self._workflow_mgr.stop_processor()
            self._server.close()
