import unittest
from mongoengine import Document, fields, connection, ConnectionFailure
from archemist.core.persistence.db_handler import DatabaseHandler


class DummyDocumentA(Document):
    a_field = fields.StringField(required=True)
    meta = {'collection': 'docs_A', 'db_alias': 'archemist_state'}


class DummyDocumentB(Document):
    a_field = fields.StringField(required=True)
    meta = {'collection': 'docs_B', 'db_alias': 'archemist_state'}


class TestDatabaseHandler(unittest.TestCase):

    def setUp(self) -> None:
        try:
            self.db_handler = DatabaseHandler(db_name="test_db", host="mongodb://localhost:27017")
        except ConnectionFailure:
            # need this to clear the cache and re-establish the connection
            # with the db
            connection._connections = {}
            connection._connection_settings = {}
            connection._dbs = {}

            DummyDocumentA._collection = None
            DummyDocumentB._collection = None
            self.db_handler = DatabaseHandler(db_name="test_db", host="mongodb://localhost:27017")

    def tearDown(self) -> None:
        # needed to properly close db connection so other tests can run
        # check (https://stackoverflow.com/questions/49390825/using-mongoengine-with-multiprocessing-how-do-you-close-mongoengine-connection)
        self.db_handler._client.close()
        self.db_handler._client = None

        connection._connections = {}
        connection._connection_settings = {}
        connection._dbs = {}

        DummyDocumentA._collection = None
        DummyDocumentB._collection = None

    def test_db_methods(self):

        self.assertFalse(self.db_handler.is_database_existing())

        # populate the db
        doc = DummyDocumentA()
        doc.a_field = "some_text"
        doc.save()
        doc = DummyDocumentB()
        doc.a_field = "some_text"
        doc.save()

        self.assertTrue(self.db_handler.is_database_existing())
        self.assertTrue(self.db_handler.is_collection_populated("docs_A"))
        self.assertTrue(self.db_handler.is_collection_populated("docs_B"))

        # clear collection
        self.db_handler.clear_collection("docs_A")
        self.assertTrue(self.db_handler.is_database_existing())
        self.assertFalse(self.db_handler.is_collection_populated("docs_A"))
        self.assertTrue(self.db_handler.is_collection_populated("docs_B"))

        # delete db
        self.db_handler.delete_database()
        self.assertFalse(self.db_handler.is_database_existing())
