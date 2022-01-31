from pymongo import MongoClient
from archemist.persistence.objectConstructor import ObjectConstructor
from bson.objectid import ObjectId
from multipledispatch import dispatch


class State:
    def __init__(self, db_name: str):
        self._mongo_client = MongoClient("mongodb://localhost:27017")
        self._db_name = db_name
        self._stations = self._mongo_client[db_name]['stations']
        self._robots = self._mongo_client[db_name]['robots']
        self._materials = self._mongo_client[db_name]['materials']
        self._batches= self._mongo_client[db_name]['batches']

    @property
    def db_name(self):
        return self._db_name
    
    @property
    def liquids(self):
        liquids = list()
        liquids_cursor = self._materials.find({'class': 'Liquid'})
        for doc in liquids_cursor:
            liquids.append(ObjectConstructor.construct_material_from_object_id(self._db_name, doc))
        return liquids

    @property
    def solids(self):
        solids = list()
        solids_cursor = self._materials.find({'class': 'Solid'})
        for doc in solids_cursor:
            solids.append(ObjectConstructor.construct_material_from_object_id(self._db_name, doc))
        return solids

    @property
    def stations(self):
        stations = list()
        stations_cursor = self._stations.find({})
        for doc in stations_cursor:
            stations.append(ObjectConstructor.construct_station_from_object_id(self._db_name, doc))
        return stations

    @property
    def robots(self):
        robots = list()
        robots_cursor = self._robots.find({})
        for doc in robots_cursor:
            robots.append(ObjectConstructor.construct_robot_from_object_id(self._db_name, doc))
        return robots

    @property
    def batches(self):
        batches = list()
        batches_cursor = self._batches.find({})
        for doc in batches_cursor:
            batches.append(ObjectConstructor.construct_batch_from_object_id(self._db_name, doc))
        return batches

    @property
    def completed_batches(self):
        processed_batches = list()
        batches_cursor = self._batches.find({'processed': True})
        for doc in batches_cursor:
            processed_batches.append(ObjectConstructor.construct_batch_from_object_id(self._db_name, doc))
        return processed_batches

    @dispatch(ObjectId)
    def get_station(self, object_id: ObjectId):
        station_doc = self._stations.find_one({'_id': object_id})
        return ObjectConstructor.construct_station_from_object_id(self._db_name, station_doc)

    @dispatch(str, int)
    def get_station(self, station_class: str, station_id: int):
        station_doc = self._stations.find_one({'class': station_class, 'id': station_id})
        return ObjectConstructor.construct_station_from_object_id(self._db_name, station_doc)

    @dispatch(ObjectId)
    def get_robot(self, object_id: ObjectId):
        robot_doc = self._robots.find_one({'_id': object_id})
        return ObjectConstructor.construct_robot_from_object_id(self._db_name, robot_doc)

    @dispatch(str, int)
    def get_robot(self, robot_class: str, robot_id: int):
        robot_doc = self._robots.find_one({'class': robot_class, 'id': robot_id})
        return ObjectConstructor.construct_robot_from_object_id(self._db_name, robot_doc)
        
        

