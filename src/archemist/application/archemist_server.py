from time import sleep
from archemist.core.processing.scheduler import PriorityQueueRobotScheduler
from archemist.core.processing.workflow_manager import WorkflowManager
from archemist.core.persistence.persistence_manager import PersistenceManager
from archemist.core.persistence.objects_getter import RobotsGetter, StationsGetter
from archemist.core.util.enums import WorkflowManagerStatus
from pathlib import Path
from archemist.application.cmd_message import CMDCategory,CMDMessage
import zmq
import json

class ArchemistServer:
    def __init__(self, workflow_dir: Path, existing_db: bool=False) -> None:
        server_config_file_path = workflow_dir.joinpath(f'config_files/server_settings.yaml')
        workflow_config_file_path = workflow_dir.joinpath(f'config_files/workflow_config.yaml')
        recipes_dir_path = workflow_dir.joinpath(f'recipes')


        # construct persistence manager
        self._persistence_mgr = PersistenceManager(server_config_file_path)
        
        # construct workflow
        if not existing_db:
            self._log_server('constructing new workflow state from config file')
            in_state, wf_state, out_state = self._persistence_mgr.construct_workflow_from_config_file(workflow_config_file_path)
        else:
            self._log_server('reconstructing workflow state from existing database')
            in_state, wf_state, out_state = self._persistence_mgr.construct_workflow_from_db()
        
        # construct workflow manager
        robot_scheduler = PriorityQueueRobotScheduler()
        self._workflow_mgr = WorkflowManager(in_state, wf_state, out_state, robot_scheduler, recipes_dir_path)

        # start ARChemist server
        context = zmq.Context()
        self._server = context.socket(zmq.PAIR)
        self._server.bind('tcp://127.0.0.1:5555')
        
        self._log_server(f'ARChemist server started')


    def run(self):
        # spin
        while True:
            try:
                json_msg = self._server.recv_json(flags=zmq.NOBLOCK)
                msg = CMDMessage.from_json(json_msg)
                if msg.category == CMDCategory.WORKFLOW:
                    if msg.cmd == 'start':
                        if self._workflow_mgr.status == WorkflowManagerStatus.INVALID:
                            self._workflow_mgr.start()
                        elif self._workflow_mgr.status == WorkflowManagerStatus.PAUSED:
                            self._workflow_mgr.resume()
                    elif msg.cmd == 'pause':
                        self._workflow_mgr.pause()
                    elif msg.cmd == 'add_batch':
                        self._workflow_mgr.add_clean_batch()
                    elif msg.cmd == 'remove_lot':
                        lot_slot = msg.params[0]
                        self._workflow_mgr.remove_lot(lot_slot)
                    elif msg.cmd == 'remove_all_lots':
                        self._workflow_mgr.remove_all_lots()
                    elif msg.cmd == 'terminate':
                        self.shut_down()
                        break
                elif msg.category == CMDCategory.ROBOT:
                    if msg.cmd == 'get_list':
                        robots = RobotsGetter.get_robots()
                        robots_dict = {
                            'names': [robot.__class__.__name__ for robot in robots],
                            'ids':  [robot.id for robot in robots],
                        }
                        json_msg = json.dumps(robots_dict)
                        self._server.send_json(json_msg)
                    elif msg.cmd == 'repeat_op':
                        robot = RobotsGetter.get_robot(msg.params[0], msg.params[1])
                        robot.repeat_assigned_op()
                    elif msg.cmd == 'skip_op':
                        robot = RobotsGetter.get_robot(msg.params[0], msg.params[1])
                        robot.skip_assigned_op()
                elif msg.category == CMDCategory.STATION:
                    if msg.cmd == 'get_list':
                        stations = StationsGetter.get_stations()
                        stations_dict = {
                            'names': [station.__class__.__name__ for station in stations],
                            'ids':  [station.id for station in stations],
                        }
                        json_msg = json.dumps(stations_dict)
                        self._server.send_json(json_msg)
                    elif msg.cmd == 'repeat_op':
                        station = StationsGetter.get_station(msg.params[0], msg.params[1])
                        station.repeat_assigned_op()
                    elif msg.cmd == 'skip_op':
                        station = StationsGetter.get_station(msg.params[0], msg.params[1])
                        station.skip_assigned_op()
            except zmq.ZMQError:
                sleep(0.2)
            except KeyboardInterrupt:
                self.shut_down()
                break

    def shut_down(self):
        if self._workflow_mgr.status != WorkflowManagerStatus.INVALID:
            self._workflow_mgr.terminate()
        self._server.close()

    def _log_server(self, message:str):
        print(f'[{self.__class__.__name__}]: {message}')
