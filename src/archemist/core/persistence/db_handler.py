from mongoengine import connect

class DatabaseHandler:
    def __init__(self, host: str, db_name: str):
        self._client = connect(db=db_name, host=host, alias='archemist_state')
        self._db_name = db_name

    def clear_collection(self, collection: str):
        if self.is_database_existing():
                if collection in self._client[self._db_name].list_collection_names():
                    self._client[self._db_name][collection].drop()

    def is_collection_populated(self, collection: str):
        if self.is_database_existing():
            return collection in self._client[self._db_name].list_collection_names()

    def delete_database(self):
        if self.is_database_existing():
            self._client.drop_database(self._db_name)

    def is_database_existing(self):
        return self._db_name in self._client.list_database_names()