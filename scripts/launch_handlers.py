#!/usr/bin/env python3

from archemist.core.persistence.yaml_handler import YamlHandler
from archemist.core.persistence.persistence_manager import PersistenceManager
from archemist.core.persistence.object_factory import StationFactory, RobotFactory
import multiprocessing as mp
from pathlib import Path
import argparse
import time
import sys


def run_station_handler(db_host, db_name, station, use_sim_handler):
    # Required to establish connection with db
    p_manager = PersistenceManager(db_host, db_name)
    # Required to load db models
    p_manager.load_station_models()
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


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Launch workflow handlers")
    parser.add_argument(
        "--sim",
        dest="sim_mode",
        action="store_true",
        help="run the given recipe continuously in test mode",
    )
    parser.add_argument(
        "--path",
        dest="workflow_dir",
        action="store",
        type=str,
        help="path to the workflow directory",
        required=True,
    )
    parser.add_argument(
        "--timeout",
        dest="timeout",
        action="store",
        type=int,
        default=5,
        help="timeout for db state to be available",
    )
    args = parser.parse_args()

    workflow_dir = Path(args.workflow_dir)

    # FIXME f"" was used before, but with no placeholder?
    server_config_file_path = workflow_dir.joinpath("config_files/server_settings.yaml")
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
        # Define robot handlers processes
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
        # Launch handlers this assumes the state was
        # constructed from a config file before hand
        processes = robot_handlers_processes + station_handlers_processes
        for process in processes:
            process.daemon = True
            process.start()
        while any(p.is_alive() for p in processes):
            time.sleep(0.1)
    except KeyboardInterrupt:
        for process in processes:
            process.terminate()
            process.join()
