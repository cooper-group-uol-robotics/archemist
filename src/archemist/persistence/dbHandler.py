from pymongo import MongoClient
from archemist.persistence.yamlHandler import YamlHandler
import os
from datetime import datetime


class dbHandler:
    def __init__(self):
        self._client = MongoClient("mongodb://localhost:27017")

    def clear_database(self, db_name:str):
        if self.is_database_populated(db_name):
            coll_list = self._client[db_name].list_collection_names()
            for coll in coll_list:
                self._client[db_name][coll].drop()

    def is_database_existing(self, db_name: str):
        db_list = self._client.list_database_names()
        return db_name in db_list

    def is_database_populated(self, db_name: str):
        if self.is_database_existing(db_name):
            coll_list = self._client[db_name].list_collection_names()
            if len(coll_list) == 4:
                return True
        return False

    def delete_database(self, db_name:str):
        if self.is_database_existing(db_name):
            self._client.drop_database(db_name)
        

    

