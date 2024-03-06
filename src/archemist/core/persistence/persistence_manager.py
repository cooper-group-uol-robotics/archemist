from archemist.core.persistence.db_handler import DatabaseHandler
from archemist.core.persistence.yaml_handler import YamlHandler
from archemist.core.persistence.object_factory import RobotFactory, StationFactory
from archemist.core.persistence.objects_getter import StateGetter
from archemist.core.state.material import Liquid, Solid
from archemist.core.state.state import InputState, WorkflowState, OutputState
from archemist.core.exceptions.exception import DatabaseNotPopulatedError
from pathlib import Path
from typing import Tuple


class PersistenceManager:
    def __init__(self, server_config_path: Path):
        server_config = YamlHandler.load_server_settings_file(server_config_path)
        db_name = server_config["db_name"]
        db_host = server_config["mongodb_host"]
        self._db_handler = DatabaseHandler(db_host, db_name)

    def construct_workflow_from_config_file(self, config_file_path: Path) -> Tuple[InputState, WorkflowState, OutputState]:
        if self._db_handler.is_database_existing():
            self._log("Database already existing")
            self._log("Deleting Database")
            self._db_handler.delete_database()

        config_dict = YamlHandler.load_config_file(config_file_path)
        if 'robots' in config_dict:
            for robot_dict in config_dict['robots']:
                RobotFactory.create_from_dict(robot_dict)

        if 'stations' in config_dict:
            for station_dict in config_dict['stations']:
                StationFactory.create_from_dict(station_dict)

        input_state = InputState.from_dict(config_dict['workflow_input'])
        workflow_state = WorkflowState.from_args(config_dict['general']['name'])
        output_state = OutputState.from_dict(config_dict['workflow_output'])

        self._log("Workflow constructed from config file")

        return input_state, workflow_state, output_state

    def construct_workflow_from_db(self) -> Tuple[InputState, WorkflowState, OutputState]:
        if self._db_handler.is_database_existing():
            input_state = StateGetter.get_input_state()
            workflow_state = StateGetter.get_workflow_state()
            output_state = StateGetter.get_output_state()

            self._log("Workflow constructed from existing database")

            return input_state, workflow_state, output_state
        else:
            raise DatabaseNotPopulatedError()

    def _log(self, message: str):
        print(f'[{self.__class__.__name__}]: {message}')
