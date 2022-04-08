from archemist.persistence.dbHandler import dbHandler
from archemist.persistence.yamlHandler import YamlHandler
from archemist.persistence.objectConstructor import ObjectConstructor
from archemist.state.state import State
from archemist.exceptions.exception import DatabaseNotPopulatedError


class PersistenceManager:
    def __init__(self, db_name: str):
        self._db_name = db_name
        self._dbhandler = dbHandler()
        
    def construct_state_from_config_file(self, config_file_path:str):
        self._dbhandler.clear_database(self._db_name)
        
        config_dict = YamlHandler.loadYamlFile(config_file_path)
        if 'Robots' in config_dict['workflow']:
            for robot_dict in config_dict['workflow']['Robots']:
                ObjectConstructor.construct_robot_from_document(self._db_name, robot_dict)

        liquids = list()
        solids = list()
        
        if config_dict['workflow']['Materials'] is not None:
            if 'liquids' in config_dict['workflow']['Materials']:
                for liquid_doc in config_dict['workflow']['Materials']['liquids']:
                    liquids.append(ObjectConstructor.construct_material_from_document(self._db_name, 'Liquid', liquid_doc))

            
            if 'solids' in config_dict['workflow']['Materials']:
                for solid_doc in config_dict['workflow']['Materials']['solids']:
                    solids.append(ObjectConstructor.construct_material_from_document(self._db_name, 'Solid', solid_doc))

        for station_dict in config_dict['workflow']['Stations']:
            ObjectConstructor.construct_station_from_document(self._db_name, station_dict, liquids, solids)

        return State(self._db_name)

    def construct_state_from_db(self):
        if self._dbhandler.is_database_populated(self._db_name):
            return State(self._db_name)
        else:
            raise DatabaseNotPopulatedError()

    
