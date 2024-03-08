from archemist.application.archemist_server import ArchemistServer
from archemist.application.archemist_cli import ArchemistCLI
from archemist.core.persistence.yaml_handler import YamlHandler
from archemist.core.processing.handler import StationHandler, RobotHandler
from archemist.core.persistence.object_factory import StationFactory, RobotFactory
from archemist.core.persistence.objects_getter import StationsGetter, RobotsGetter
from archemist.core.persistence.db_handler import DatabaseHandler
import multiprocessing as mp
from pathlib import Path
import argparse
import time
import sys


def run_cli(args):
    cli_client = ArchemistCLI()
    cli_client.run()


def run_local_server(args):
    workflow_dir = Path(args.workflow_path)
    if not workflow_dir.exists():
        print('The given workflow directory does not exist.')
        print('Please check your path or create a new directory using create_workflow_dir script')
        exit()
    existing_db = args.existing_db
    server = ArchemistServer(workflow_dir, existing_db)
    server.run()


def run_station_handler(db_host, db_name, station_object_id, use_sim_handler):
    # needed to establish connection with db
    db_handler = DatabaseHandler(db_host, db_name)
    station = StationFactory.create_from_object_id(station_object_id)
    handler = StationHandler(station, use_sim_handler)
    handler.initialise()
    handler.run()


def run_robot_handler(db_host, db_name, robot_object_id, use_sim_handler):
    # needed to establish connection with db
    db_handler = DatabaseHandler(db_host, db_name)
    robot = RobotFactory.create_from_object_id(robot_object_id)
    handler = RobotHandler(robot, use_sim_handler)
    handler.initialise()
    handler.run()


def launch_handler(args):

    workflow_dir = Path(args.workflow_dir)

    server_config_file_path = workflow_dir.joinpath(
        f'config_files/server_settings.yaml')
    server_settings = YamlHandler.load_server_settings_file(
        server_config_file_path)
    db_name = server_settings['db_name']
    db_host = server_settings['mongodb_host']

    try:
        db_handler = DatabaseHandler(db_host, db_name)

        start_time = time.time()
        while not db_handler.is_database_existing():
            print('waiting on database state to exist')
            time.sleep(0.5)
            if time.time() - start_time > args.timeout:
                sys.exit('timeout reached! no db state is available. Exiting')

        mp.set_start_method('spawn')  # to avoid forking error with mongodb
        # define robot handlers processes
        robot_handlers_processes = []
        for robot in RobotsGetter.get_robots():
            kwargs = {
                'db_host': db_host,
                'db_name': db_name,
                'robot_object_id': robot.object_id,
                'use_sim_handler': args.sim_mode
            }
            robot_handlers_processes.append(mp.Process(
                target=run_robot_handler, kwargs=kwargs))
        # define station handlers processes
        station_handlers_processes = []
        for station in StationsGetter.get_stations():
            kwargs = {
                'db_host': db_host,
                'db_name': db_name,
                'station_object_id': station.object_id,
                'use_sim_handler': args.sim_mode
            }
            station_handlers_processes.append(mp.Process(
                target=run_station_handler, kwargs=kwargs))
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


def main():
    parser = argparse.ArgumentParser(
        description='Archemist command line tools', prog='ARCHemist')
    subparsers = parser.add_subparsers(help='archemist commands help')

    # start_server parser
    local_server_parser = subparsers.add_parser(
        'start_server', help='command to run ARCHemist server locally')
    local_server_parser.add_argument('-p', '--path', dest='workflow_path', action='store', type=str,
                                     help='path to the workflow directory', required=True)
    local_server_parser.add_argument('--exists', dest='existing_db', action='store_true',
                                     help='run server with already existing database')
    local_server_parser.set_defaults(func=run_local_server)

    # run_cli client parser
    local_cli_parser = subparsers.add_parser(
        'start_cli', help='command to start ARCHemist CLI')
    local_cli_parser.set_defaults(func=run_cli)

    # launch handlers parser
    launch_handlers_parser = subparsers.add_parser(
        'launch_handlers', help='command to Launch ARCHemist workflow handlers')
    launch_handlers_parser.add_argument('--sim', dest='sim_mode', action='store_true',
                                        help='run the given recipe continuously in test mode')
    launch_handlers_parser.add_argument('-p', '--path', dest='workflow_dir', action='store', type=str,
                                        help='path to the workflow directory', required=True)
    launch_handlers_parser.add_argument('-timeout', dest='timeout', action='store', type=int,
                                        default=5, help='timeout for db state to be available')
    launch_handlers_parser.set_defaults(func=launch_handler)

    args = parser.parse_args()
    if vars(args) != {}:
        args.func(args)
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
