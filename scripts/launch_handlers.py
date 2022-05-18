from archemist.persistence.yamlHandler import YamlHandler
from archemist.persistence.persistenceManager import PersistenceManager
from archemist.persistence.objectConstructor import ObjectConstructor
import archemist.processing.robotHandlers
import archemist.processing.stationHandlers
import multiprocessing as mp
from time import sleep
from pathlib import Path

from collections import namedtuple

HandlerArgs = namedtuple('HandlerArgs', ['type','class_name', 'id'])

def run_handler(handler_discriptor: HandlerArgs):
    p_manager = PersistenceManager('dif_demo')
    state = p_manager.construct_state_from_db()
    if handler_discriptor.type == 'stn':
        station = state.get_station(handler_discriptor.class_name, handler_discriptor.id)
        handler = construct_station_handler(station)
    elif handler_discriptor.type == 'rob':
        robot = state.get_robot(handler_discriptor.class_name, handler_discriptor.id)
        handler = construct_robot_handler(robot)
    handler.run()

def construct_robot_handler(robot):
    handler_name = f'{robot.__class__.__name__}_Handler'
    handler_cls = getattr(archemist.processing.robotHandlers, handler_name)
    return handler_cls(robot)

def construct_station_handler(station):
    handler_name = f'{station.__class__.__name__}_Handler'
    handler_cls = getattr(archemist.processing.stationHandlers, handler_name)
    return handler_cls(station)

if __name__ == '__main__':
    
    current_dir = Path.cwd()
    config_file_path = current_dir.joinpath('config_files/dif_demo_testing_config_file.yaml')

    try:
        # get config dict
        config_dict = YamlHandler.loadYamlFile(config_file_path.absolute())

        handlers_discrptors = [HandlerArgs('stn', station['class'], station['id']) for station in config_dict['workflow']['Stations']]
        handlers_discrptors.extend([HandlerArgs('rob', robot['class'], robot['id']) for robot in config_dict['workflow']['Robots']])

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