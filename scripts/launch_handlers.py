import importlib
from archemist.core.persistence.yaml_handler import YamlHandler
from archemist.core.persistence.persistence_manager import PersistenceManager
import multiprocessing as mp
from time import sleep
from pathlib import Path
from archemist.core.state.state import State
import argparse
import pkgutil

from collections import namedtuple

HandlerArgs = namedtuple('HandlerArgs', ['db_host','db_name','test_mode','type','class_name', 'id'])

def run_handler(handler_discriptor: HandlerArgs):
    p_manager = PersistenceManager(handler_discriptor.db_host, handler_discriptor.db_name)
    state = p_manager.construct_state_from_db()
    if handler_discriptor.type == 'stn':
        station = state.get_station(handler_discriptor.class_name, handler_discriptor.id)
        if handler_discriptor.test_mode:
            handler = construct_station_test_handler(station)
        else:
            handler = construct_station_handler(station)
    elif handler_discriptor.type == 'rob':
        robot = state.get_robot(handler_discriptor.class_name, handler_discriptor.id)
        if handler_discriptor.test_mode:
            handler = construct_robot_test_handler(robot)
        else:
            handler = construct_robot_handler(robot)
    handler.run()

def construct_robot_handler(robot):
    handler_name = f'{robot.__class__.__name__}_Handler'
    pkg = importlib.import_module('archemist.robots')
    for module_itr in pkgutil.iter_modules(path=pkg.__path__,prefix=f'{pkg.__name__}.'):
        handler_module = f'{module_itr.name}.handler'
        module = importlib.import_module(handler_module)
        if hasattr(module,handler_name):
            cls = getattr(module,handler_name)
            return cls(robot)

def construct_robot_test_handler(robot):
    pkg = importlib.import_module('archemist.robots.simulated_robot.handler')
    handler_cls = getattr(pkg, 'GenericRobotHandler')
    return handler_cls(robot) 

def construct_station_handler(station):
    handler_name = f'{station.__class__.__name__}_Handler'
    pkg = importlib.import_module('archemist.stations')
    for module_itr in pkgutil.iter_modules(path=pkg.__path__,prefix=f'{pkg.__name__}.'):
        handler_module = f'{module_itr.name}.handler'
        module = importlib.import_module(handler_module)
        if hasattr(module,handler_name):
            cls = getattr(module,handler_name)
            return cls(station)

def construct_station_test_handler(station):
    pkg = importlib.import_module('archemist.stations.simulated_station.handler')
    handler_cls = getattr(pkg, 'GenericStationHandler')
    return handler_cls(station)

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Launch workflow handlers')
    parser.add_argument('--t', dest='test_mode', action='store_true',
                    help='run the given recipe continously in test mode')
    args = parser.parse_args()
    
    current_dir = Path.cwd()
    server_config_file_path = current_dir.joinpath(f'server_settings.yaml')
    server_setttings = YamlHandler.loadYamlFile(server_config_file_path)

    workflow_dir = Path(server_setttings['workflow_dir_path'])
    workflow_config_file_path = workflow_dir.joinpath(f'config_files/workflow_config.yaml')
    db_name = server_setttings['db_name']

    try:
        # get config dict
        config_dict = YamlHandler.loadYamlFile(workflow_config_file_path.absolute())
        host='mongodb://localhost:27017'
        handlers_discrptors = [HandlerArgs(host,db_name, args.test_mode, 'stn', station['type'], station['id']) for station in config_dict['workflow']['Stations']]
        handlers_discrptors.extend([HandlerArgs(host,db_name, args.test_mode, 'rob', robot['type'], robot['id']) for robot in config_dict['workflow']['Robots']])
        # launch handlers this assumes the state was constructed from a config file before hand
        procs = [mp.Process(target=run_handler, args=(desciptor,)) for desciptor in handlers_discrptors]
        for proc in procs:
            proc.daemon = True
            proc.start()
        while any(proc.is_alive() for proc in procs):
            sleep(0.1)
    except KeyboardInterrupt:
        for proc in procs:
            proc.terminate()
            proc.join()