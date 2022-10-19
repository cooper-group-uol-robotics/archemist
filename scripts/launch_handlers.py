from archemist.persistence.yamlHandler import YamlHandler
from archemist.persistence.persistenceManager import PersistenceManager
import archemist.processing.robotHandlers
import archemist.processing.stationHandlers
import multiprocessing as mp
from time import sleep
from pathlib import Path
from archemist.state.state import State
import argparse

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
    handler_cls = getattr(archemist.processing.robotHandlers, handler_name)
    return handler_cls(robot)

def construct_robot_test_handler(robot):
    handler_cls = getattr(archemist.processing.robotHandlers, 'GenericRobot_Handler')
    return handler_cls(robot) 

def construct_station_handler(station):
    handler_name = f'{station.__class__.__name__}_Handler'
    handler_cls = getattr(archemist.processing.stationHandlers, handler_name)
    return handler_cls(station)

def construct_station_test_handler(station):
    handler_cls = getattr(archemist.processing.stationHandlers, 'GenericStation_Handler')
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