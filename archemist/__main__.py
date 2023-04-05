# External
import multiprocessing as mp
from pathlib import Path
import argparse
import time
import sys

# Core
from archemist.application.archemist_server import ArchemistServer
from archemist.application.archemist_cli import ArchemistCLI
from archemist.core.persistence.yaml_handler import YamlHandler
from archemist.core.persistence.persistence_manager import PersistenceManager
from archemist.core.persistence.object_factory import StationFactory, RobotFactory


def run_cli(args):
    cli_client = ArchemistCLI()
    cli_client.run()


def run_local_server(args):
    workflow_dir = Path(args.workflow_path)
    if not workflow_dir.exists():
        # FIXME Logging?
        print("The given workflow directory does not exist.")
        print(
            "Please check your path or create a new directory \
                    using create_workflow_dir script"
        )
        exit()
    existing_db = args.existing_db
    server = ArchemistServer(workflow_dir, existing_db)
    server.run()


def run_station_handler(db_host, db_name, station, use_sim_handler):
    """Runs Robot handler.
    Args:
        db_host(str?): FIXME
        db_name(str?): FIXME
        robot(str?): FIXME
        use_sim_handler(str?): FIXME
    """
    # Required to establish connection with db
    p_manager = PersistenceManager(db_host, db_name)
    p_manager.load_station_models()  # required to load db models
    handler = StationFactory.create_handler(station, use_sim_handler)
    handler.run()


def run_robot_handler(db_host, db_name, robot, use_sim_handler):
    """Runs Robot handler.
    Args:
        db_host(str?): FIXME
        db_name(str?): FIXME
        robot(str?): FIXME
        use_sim_handler(str?): FIXME
    """
    # Required to establish connection with db
    p_manager = PersistenceManager(db_host, db_name)
    # Required to load db models
    p_manager.load_robot_models()
    handler = RobotFactory.create_handler(robot, use_sim_handler)
    handler.run()


def launch_handler(args):
    """Runs Robot handler.
    Args:
        args(str?): FIXME
    """

    workflow_dir = Path(args.workflow_dir)
    # FIXME
    server_config_file_path = workflow_dir.joinpath(
        "config_files", "server_settings.yaml"
    )
    server_settings = YamlHandler.load_server_settings_file(server_config_file_path)
    db_name = server_settings["db_name"]
    db_host = server_settings["mongodb_host"]

    try:
        p_manager = PersistenceManager(db_host, db_name)
        start_time = time.time()
        while not p_manager.is_db_state_existing():
            print("waiting on database state to exist")
            time.sleep(0.5)
            if time.time() - start_time > args.timeout:
                sys.exit("timeout reached! no db state is available. Exiting")

        state = p_manager.construct_state_from_db()
        # To avoid forking error with mongodb
        mp.set_start_method("spawn")
        # define robot handlers processes
        robot_handlers_processes = []
        for robot in state.robots:
            kwargs = {
                "db_host": db_host,
                "db_name": db_name,
                "robot": robot,
                "use_sim_handler": args.sim_mode,
            }
            robot_handlers_processes.append(
                mp.Process(target=run_robot_handler, kwargs=kwargs)
            )
        # define station handlers processes
        station_handlers_processes = []
        for station in state.stations:
            kwargs = {
                "db_host": db_host,
                "db_name": db_name,
                "station": station,
                "use_sim_handler": args.sim_mode,
            }
            station_handlers_processes.append(
                mp.Process(target=run_station_handler, kwargs=kwargs)
            )
        # launch handlers this assumes the state was constructed from a
        # config file before hand
        processes = robot_handlers_processes + station_handlers_processes
        for processe in processes:
            processe.daemon = True
            processe.start()
        while any(processe.is_alive() for processe in processes):
            time.sleep(0.1)
    except KeyboardInterrupt:
        for processe in processes:
            processe.terminate()
            processe.join()


def main():
    parser = argparse.ArgumentParser(
        description="Archemist command line tools", prog="ARCHemist"
    )
    subparsers = parser.add_subparsers(help="archemist commands help")

    # start_server parser
    local_server_parser = subparsers.add_parser(
        "start_server", help="command to run ARCHemist server locally"
    )
    local_server_parser.add_argument(
        "-path",
        dest="workflow_path",
        action="store",
        type=str,
        help="path to the workflow directory",
        required=True,
    )
    local_server_parser.add_argument(
        "--exists",
        dest="existing_db",
        action="store_true",
        help="run server with already existing database",
    )
    local_server_parser.set_defaults(func=run_local_server)

    # run_cli client parser
    local_cli_parser = subparsers.add_parser(
        "start_cli", help="command to start ARCHemist CLI"
    )
    local_cli_parser.set_defaults(func=run_cli)

    # launch handlers parser
    launch_handlers_parser = subparsers.add_parser(
        "launch_handlers", help="command to Launch ARCHemist workflow handlers"
    )
    launch_handlers_parser.add_argument(
        "--sim",
        dest="sim_mode",
        action="store_true",
        help="run the given recipe continuously in test mode",
    )
    launch_handlers_parser.add_argument(
        "-path",
        dest="workflow_dir",
        action="store",
        type=str,
        help="path to the workflow directory",
        required=True,
    )
    launch_handlers_parser.add_argument(
        "-timeout",
        dest="timeout",
        action="store",
        type=int,
        default=5,
        help="timeout for db state to be available",
    )
    launch_handlers_parser.set_defaults(func=launch_handler)

    args = parser.parse_args()
    if vars(args) != {}:
        args.func(args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
