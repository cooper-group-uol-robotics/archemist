from archemist.core.persistence.db_handler import DatabaseHandler
from archemist.core.persistence.yaml_handler import YamlHandler
from archemist.core.persistence.object_factory import RobotFactory, StationFactory, MaterialFactory
from archemist.core.models.state_model import StateModel
from archemist.core.state.state import State
from archemist.core.exceptions.exception import DatabaseNotPopulatedError


class PersistenceManager:
    def __init__(self, db_host: str, db_name: str):
        self._db_name = db_name
        self._dbhandler = DatabaseHandler(db_host, db_name)
        
    def construct_state_from_config_file(self, config_file_path:str):
        self._dbhandler.clear_database(self._db_name)
        
        config_dict = YamlHandler.loadYamlFile(config_file_path)
        if 'robots' in config_dict['workflow']:
            for robot_dict in config_dict['workflow']['robots']:
                RobotFactory.create_from_dict(robot_dict)

        liquids = []
        solids = []
        
        if 'materials' in config_dict['workflow']:
            if 'liquids' in config_dict['workflow']['materials']:
                for liquid_dict in config_dict['workflow']['materials']['liquids']:
                    liquids.append(MaterialFactory.create_liquid_from_dict(liquid_dict))

            
            if 'solids' in config_dict['workflow']['materials']:
                for solid_dict in config_dict['workflow']['materials']['solids']:
                    solids.append(MaterialFactory.create_solid_from_dict(solid_dict))

        if 'stations' in config_dict['workflow']:
            for station_dict in config_dict['workflow']['stations']:
                StationFactory.create_from_dict(station_dict, liquids, solids)

        return State.from_dict(config_dict['workflow']['general'])

    def construct_state_from_db(self):
            if self.is_db_state_existing():
                state_model = StateModel.objects.first()
                return State(state_model)
            else:
                raise DatabaseNotPopulatedError()

    def is_db_state_existing(self):
        return self._dbhandler.is_database_populated(self._db_name)

    

    
