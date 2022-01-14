from bson.objectid import ObjectId
from pymongo import MongoClient
import pickle

class DbObjProxy:
    def __init__(self, db: str, collection: str, obj_data):
        self._mongo_client = MongoClient("mongodb://localhost:27017")
        self._db_collection = self._mongo_client[db][collection]
        if isinstance(obj_data, dict):
            stored_document = self._db_collection.find_one({'object': obj_data['object'], 'id': obj_data['id']})
            if stored_document is None:
                self._db_id = self._db_collection.insert_one(obj_data).inserted_id
            else:
                self._db_id = stored_document['_id']
                self._db_collection.replace_one({'_id': self._unique_id}, obj_data)
        elif isinstance(obj_data, ObjectId):
            self._db_id = obj_data

    @classmethod
    def from_dict(cls, db: str, collection: str, obj_dict: dict):
        return cls(db,collection,obj_dict)

    @classmethod
    def from_objectId(cls, db: str, collection: str, obj_id: ObjectId):
        return cls(db, collection, obj_id)

    def update_field(self, key: str, value):
        self._db_collection.update_one({'_id': self._db_id}, {'$set': {key: value}})

    def get_field(self, key:str):
        return self._db_collection.find_one({'_id': self._db_id})[key]

    def increment_field(self, key:str):
        self._db_collection.update_one({'_id': self._db_id}, {'$inc': {key: 1}})

    def decrement_field(self, key:str):
        self._db_collection.update_one({'_id': self._db_id}, {'$inc': {key: -1}})


    def push_to_field_array(self, key:str, value):
        self._db_collection.update_one({'_id': self._db_id}, {'$push': {key: value}})

    def pop_from_field_array(self, key:str):
        self._db_collection.update_one({'_id': self._db_id}, {'$pop': {key: 1}})

    def encode_object(self, object):
        return pickle.dumps(object)

    def decode_object(self, object):
        return pickle.loads(object)