import unittest
from mongoengine import Document, fields, connect
from archemist.core.persistence.db_handler import DatabaseHandler

class DummyDocumentA(Document):
        a_field = fields.StringField(required=True)
        meta = {'collection': 'docs_A', 'db_alias': 'archemist_state'}

class DummyDocumentB(Document):
        a_field = fields.StringField(required=True)
        meta = {'collection': 'docs_B', 'db_alias': 'archemist_state'}

class TestDatabaseHandler(unittest.TestCase):

    def test_db_methods(self):
        db_handler = DatabaseHandler(db_name="test_db", host="mongodb://localhost:27017")

        self.assertFalse(db_handler.is_database_existing())

        # populate the db
        doc = DummyDocumentA()
        doc.a_field = "some_text"
        doc.save()
        doc = DummyDocumentB()
        doc.a_field = "some_text"
        doc.save()
        
        self.assertTrue(db_handler.is_database_existing())
        self.assertTrue(db_handler.is_collection_populated("docs_A"))
        self.assertTrue(db_handler.is_collection_populated("docs_B"))

        # clear collection
        db_handler.clear_collection("docs_A")
        self.assertTrue(db_handler.is_database_existing())
        self.assertFalse(db_handler.is_collection_populated("docs_A"))
        self.assertTrue(db_handler.is_collection_populated("docs_B"))

        # delete db
        db_handler.delete_database()
        self.assertFalse(db_handler.is_database_existing())