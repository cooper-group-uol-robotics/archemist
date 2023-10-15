import unittest

from archemist.core.state.batch import Batch
from archemist.core.state.recipe import Recipe
from archemist.core.state.lot import Lot
from archemist.core.util.location import Location
from archemist.core.util.enums import LotStatus
from mongoengine import connect

class LotTest(unittest.TestCase):
    def setUp(self) -> None:
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        self.recipe_doc = {
            "general": {"name": "test_archemist_recipe", "id": 198},
            "steps": [
                {
                    "state_name": "stirring_operation",
                    "station": {
                        "type": "IkaPlateDigital",
                        "id": 2,
                        "process": {
                            "type": "CrystalBotWorkflowProcess",
                            "operations": [
                                {
                                    "name": "stir",
                                    "type": "IKAStirringOpDescriptor",
                                    "repeat_for_all_batches": False,
                                    "parameters": [
                                        {
                                            "stirring_speed": 200,
                                            "duration": 10,
                                        },
                                        {
                                            "stirring_speed": 150,
                                            "duration": 5,
                                        },
                                    ],
                                },
                            ],
                            "args": None,
                        },
                    },
                    "transitions": {
                        "on_success": "weighing_operation",
                        "on_fail": "failed_state",
                    },
                },
                {
                    "state_name": "weighing_operation",
                    "station": {
                        "type": "FisherWeightingStation",
                        "id": 5,
                        "process": {
                            "type": "SomeProcess",
                            "operations": [
                                {
                                    "name": "weigh",
                                    "type": "FisherWeightOpDescriptor",
                                    "repeat_for_all_batches": True,
                                    "parameters": [{"some_param": 123}],
                                },
                            ],
                            "args": {"some_variable": 42},
                        },
                    },
                    "transitions": {
                        "on_success": "end_state",
                        "on_fail": "failed_state",
                    },
                },
            ],
        }

    def  tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_lot(self):
        # test construction
        batch_1 = Batch.from_args(3, Location(1, 2, "some_frame"))
        batch_2 = Batch.from_args(3, Location(1, 2, "some_frame"))
        lot = Lot.from_args([batch_1, batch_2])
        self.assertIsNotNone(lot.object_id)
        self.assertEqual(lot.status, LotStatus.CREATED)
        lot.status = LotStatus.FINISHED
        self.assertEqual(lot.status, LotStatus.FINISHED)
        self.assertEqual(len(lot.batches), 2)
        self.assertEqual(lot.num_batches, 2)
        self.assertEqual(lot.batches[0], batch_1)
        self.assertEqual(lot.batches[1], batch_2)
        self.assertEqual(lot.batches[0].parent_lot_id, lot.object_id)
        lot.add_station_stamp("test_station_stamp")
        for batch in lot.batches:
            self.assertEqual(len(batch.station_stamps), 1)
            self.assertTrue("test_station_stamp" in batch.station_stamps[0])
        
        # test recipe
        self.assertFalse(lot.is_recipe_attached())
        self.assertIsNone(lot.recipe)
        recipe = Recipe.from_dict(self.recipe_doc)
        lot.attach_recipe(recipe)
        self.assertTrue(lot.is_recipe_attached())
        self.assertIsNotNone(lot.recipe)

        # test construction from object id
        lot_copy = Lot.from_object_id(lot.object_id)
        self.assertEqual(lot, lot_copy)

if __name__ == "__main__":
    unittest.main()