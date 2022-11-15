from archemist.core.persistence.yaml_handler import YamlHandler
from archemist.core.persistence.persistence_manager import PersistenceManager
from archemist.core.persistence.object_factory import StationFactory, RobotFactory
import multiprocessing as mp
from pathlib import Path
import argparse
import time
import sys


def run_station_handler(db_host, db_name, station, use_sim_handler):
    p_manager = PersistenceManager(db_host, db_name) # required to establish connection with db
    p_manager.load_station_models()  # required to load db models
    handler = StationFactory.create_handler(station, use_sim_handler)
    handler.run()

def run_robot_handler(db_host, db_name, robot, use_sim_handler):
    p_manager = PersistenceManager(db_host, db_name) # required to establish connection with db
    p_manager.load_robot_models() # required to load db models
    handler = RobotFactory.create_handler(robot, use_sim_handler)
    handler.run()

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Launch workflow handlers')
    parser.add_argument('--sim', dest='sim_mode', action='store_true',
                    help='run the given recipe continuously in test mode')
    parser.add_argument('--conf', dest='config_dir', action='store', type=str,
                    help='path to the config directory')
    parser.add_argument('--timeout', dest='timeout', action='store', type=int,
                    default=5, help='timeout for db state to be available')
    args = parser.parse_args()

    if args.config_dir is None:
        config_dir = Path.cwd()
    else:
        config_dir = Path(args.config_dir)
    
    server_config_file_path = config_dir.joinpath(f'server_settings.yaml')
    server_settings = YamlHandler.load_server_settings_file(server_config_file_path)
    db_name = server_settings['db_name']
    db_host = server_settings['mongodb_host']

    try:
        p_manager = PersistenceManager(db_host, db_name)

        start_time = time.time()
        while not p_manager.is_db_state_existing():
            print('waiting on database state to exist')
            time.sleep(0.5)
            if time.time() - start_time > args.timeout:
                sys.exit('timeout reached! no db state is available. Exiting')

        state = p_manager.construct_state_from_db()
        # define robot handlers processes
        robot_handlers_processes = []
        for robot in state.robots:
            kwargs = {
                'db_host': db_host,
                'db_name': db_name,
                'robot': robot,
                'use_sim_handler': args.sim_mode
                }
            robot_handlers_processes.append(mp.Process(target=run_robot_handler, kwargs=kwargs))
        # define station handlers processes
        station_handlers_processes = []
        for station in state.stations:
            kwargs = {
                'db_host': db_host,
                'db_name': db_name,
                'station': station,
                'use_sim_handler': args.sim_mode
                }
            station_handlers_processes.append(mp.Process(target=run_station_handler, kwargs=kwargs))
        # launch handlers this assumes the state was constructed from a config file before hand
        processes = robot_handlers_processes + station_handlers_processes
        for p in processes:
            p.daemon = True
            p.start()
        while any(p.is_alive() for p in processes):
            time.sleep(0.1)
    except KeyboardInterrupt:
        for p in processes:
            p.terminate()
            p.join()