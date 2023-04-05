from mongoengine import connect


class DatabaseHandler:
    """Description FIXME"""

    def __init__(self, host: str, db_name: str):
        self._client = connect(db=db_name, host=host, alias="archemist_state")

    def clear_database(self, db_name: str):
        """Description FIXME"""
        if self.is_database_populated(db_name):
            coll_list = self._client[db_name].list_collection_names()
            for coll in coll_list:
                self._client[db_name][coll].drop()

    def is_database_existing(self, db_name: str):
        """Description FIXME"""
        db_list = self._client.list_database_names()
        return db_name in db_list

    def is_database_populated(self, db_name: str):
        """Description FIXME"""
        if self.is_database_existing(db_name):
            coll_list = self._client[db_name].list_collection_names()
            if len(coll_list) > 0:
                return True
        return False

    def delete_database(self, db_name: str):
        """Description FIXME"""
        if self.is_database_existing(db_name):
            self._client.drop_database(db_name)
