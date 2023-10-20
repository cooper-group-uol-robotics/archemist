import unittest
from pathlib import Path

from archemist.core.persistence.persistence_manager import PersistenceManager, DatabaseNotPopulatedError
from archemist.core.persistence.objects_getter import RobotsGetter, StationsGetter, MaterialsGetter

class PersistenceManagerTest(unittest.TestCase):

    def setUp(self):
        db_config_path = Path.joinpath(Path.cwd(), "tests/persistence/resources/good_server_config.yaml")
        self.p_manager = PersistenceManager(db_config_path)

    def tearDown(self) -> None:
        self.p_manager._db_handler.delete_database()

    def test_workflow_construction(self):
        # test constructing workflow from non existing database
        with self.assertRaises(DatabaseNotPopulatedError):
            self.p_manager.construct_workflow_from_db()
        
        # test constructing workflow from config file
        workflow_config_path = Path.joinpath(Path.cwd(), "tests/persistence/resources/good_wf_config.yaml")
        
        in_state, wf_state, out_state = self.p_manager.construct_workflow_from_config_file(workflow_config_path)
        self.assertIsNotNone(in_state)
        self.assertIsNotNone(wf_state)
        self.assertIsNotNone(out_state)
        
        self.assertEqual(len(StationsGetter.get_stations()), 2)
        self.assertEqual(len(RobotsGetter.get_robots()), 2)
        self.assertEqual(len(MaterialsGetter.get_solids()), 1)
        self.assertEqual(len(MaterialsGetter.get_liquids()), 1)

        # test constructing workflow from existing database
        in_state_db, wf_state_db, out_state_db = self.p_manager.construct_workflow_from_db()
        self.assertEqual(in_state_db.object_id, in_state.object_id)
        self.assertEqual(wf_state_db.object_id, wf_state.object_id)
        self.assertEqual(out_state_db.object_id, out_state.object_id)