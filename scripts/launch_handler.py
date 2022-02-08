from archemist.persistence.yamlHandler import YamlHandler
from archemist.persistence.persistenceManager import PersistenceManager
from archemist.persistence.objectConstructor import ObjectConstructor
import multiprocessing as mp
from time import sleep
import os

from collections import namedtuple

HanlderDescriptor = namedtuple('HanlderArgs', ['type','class_name', 'id'])

def run_handler(handler_discriptor: HanlderDescriptor):
    p_manager = PersistenceManager('test')
    state = p_manager.construct_state_from_db()
    
    if handler_discriptor.type == 'stn':
        station = state.get_station(handler_discriptor.class_name, handler_discriptor.id)
        handler = ObjectConstructor.construct_station_handler(station)
    elif handler_discriptor.type == 'rob':
        robot = state.get_robot(handler_discriptor.class_name, handler_discriptor.id)
        handler = ObjectConstructor.construct_robot_handler(robot)
    handler.run()

if __name__ == '__main__':
    
    current_dir = os.path.dirname(os.path.realpath(__file__))
    config_file_path = os.path.abspath(os.path.join(current_dir, '../state/resources/testing_config_file.yaml'))

    try:
        # get config dict
        config_dict = YamlHandler.loadYamlFile(config_file_path)
        handlers_discrptors = [('stn', station['class'], station['id']) for station in config_dict['Stations']]
        handlers_discrptors.extend([('rob', robot['class'], robot['id']) for robot in config_dict['Robots']])
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
    finally:
        print('All processes terminated')
