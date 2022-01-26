from archemist.state.batch import Batch
from archemist.persistence.dbHandler import dbHandler
import archemist.state.robots
import archemist.state.stations
from archemist.state.station import Station
import archemist.state.material
import archemist.processing.stationSMs


class ObjectConstructor:

    @staticmethod
    def construct_station_from_document(db_name: str, station_dict: dict, liquids: list, solids: list):
        station_cls = getattr(archemist.state.stations, station_dict['class'])
        return station_cls.from_dict(db_name, station_dict, liquids, solids)

    @staticmethod
    def construct_station_from_object_id(db_name: str, station_document: dict):
        station_cls = getattr(archemist.state.stations, station_document['class'])
        return station_cls.from_object_id(db_name, station_document['_id'])

    @staticmethod
    def construct_material_from_document(db_name: str, material_class: str, material_dict: dict):
        material_cls = getattr(archemist.state.material, material_class)
        return material_cls.from_dict(db_name, material_dict)

    @staticmethod
    def construct_material_from_object_id(db_name: str, material_document: dict):
        material_cls = getattr(archemist.state.material, material_document['class'])
        return material_cls.from_object_id(db_name, material_document['_id'])

    @staticmethod
    def construct_robot_from_document(db_name: str, robot_document: dict):
        robot_cls = getattr(archemist.state.robots, robot_document['class'])
        return robot_cls.from_dict(db_name, robot_document)

    @staticmethod
    def construct_robot_from_object_id(db_name: str, robot_document: dict):
        robot_cls = getattr(archemist.state.robots, robot_document['class'])
        return robot_cls.from_object_id(db_name, robot_document['_id'])

    @staticmethod
    def construct_batch_from_object_id(db_name: str, batch_document: dict):
        return Batch.from_object_id(db_name, batch_document['_id'])

    @staticmethod
    def construct_process_sm_for_station(station: Station):
        sm_dict = station.process_sm_dict
        station_sm_cls = getattr(archemist.processing.stationSMs, sm_dict['type'])
        return station_sm_cls(station, sm_dict['args'])
