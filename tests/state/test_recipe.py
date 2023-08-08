import unittest
from archemist.stations.ika_digital_plate_station.state import (
    IKAStirringOpDescriptor,
)
from archemist.stations.fisher_balance_station.state import FisherWeightOpDescriptor
from archemist.core.state.recipe import Recipe
from mongoengine import connect


class RecipeTest(unittest.TestCase):
    def setUp(self) -> None:
        self._db_name = "archemist_test"
        self._client = connect(
            db=self._db_name, host="mongodb://localhost:27017", alias="archemist_state"
        )

    def tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_recipe(self):
        
        recipe_doc = {
            "general": {"name": "test_archemist_recipe", "id": 198},
            "materials": {
                "liquids": [{"name": "water", "id": 1}],
                "solids": [{"name": "sodium_chloride", "id": 2}],
            },
            "process": [
                {
                    "state_name": "stirring_operation",
                    "station": {
                        "type": "IkaPlateDigital",
                        "id": 2,
                        "process": {"type": "CrystalBotWorkflowProcess", "args": None},
                        "operation": {
                            "type": "IKAStirringOpDescriptor",
                            "properties": {"stirring_speed": 200, "duration": 10},
                        },
                    },
                    "transitions": {
                        "on_success": "weighing_operation",
                        "on_fail": "end_state",
                    },
                },
                {
                    "state_name": "weighing_operation",
                    "station": {
                        "type": "FisherWeightingStation",
                        "id": 5,
                        "process": {
                            "type": "SomeProcess",
                            "args": {"some_variable": 42},
                        },
                        "operation": {"type": "FisherWeightOpDescriptor"},
                    },
                    "transitions": {"on_success": "end_state", "on_fail": "end_state"},
                },
            ],
        }

        """create recipe"""
        recipe = Recipe.from_dict(recipe_doc)

        self.assertEqual(recipe.id, 198)
        self.assertEqual(recipe.name, "test_archemist_recipe")
        self.assertEqual(recipe.liquids[0], {"name": "water", "id": 1})
        self.assertEqual(recipe.solids[0], {"name": "sodium_chloride", "id": 2})

        # stirring state
        self.assertEqual(recipe.current_state, "stirring_operation")
        station_name, station_id = recipe.get_current_station()
        self.assertEqual(station_name, "IkaPlateDigital")
        self.assertEqual(station_id, 2)
        process_1 = recipe.get_current_process()
        self.assertEqual(process_1["type"], "CrystalBotWorkflowProcess")
        self.assertIsNone(process_1["args"])
        op_1 = recipe.get_current_task_op()
        self.assertTrue(isinstance(op_1, IKAStirringOpDescriptor))
        self.assertEqual(op_1.target_stirring_speed, 200)
        self.assertEqual(op_1.target_duration, 10)
        self.assertFalse(recipe.is_complete())
        next_station, next_station_id = recipe.get_next_station(success=True)
        self.assertEqual(next_station, "FisherWeightingStation")
        self.assertEqual(next_station_id, 5)

        # weighing state
        recipe.advance_state(True)
        self.assertEqual(recipe.current_state, "weighing_operation")
        station_name, station_id = recipe.get_current_station()
        self.assertEqual(station_name, "FisherWeightingStation")
        self.assertEqual(station_id, 5)
        process_2 = recipe.get_current_process()
        self.assertEqual(process_2["type"], "SomeProcess")
        self.assertEqual(process_2["args"], {"some_variable": 42})
        op_2 = recipe.get_current_task_op()
        self.assertTrue(isinstance(op_2, FisherWeightOpDescriptor))
        self.assertFalse(recipe.is_complete())
        next_station, next_station_id = recipe.get_next_station(success=True)
        self.assertEqual(next_station, "end")
        self.assertIsNone(next_station_id)
        # end state
        recipe.advance_state(True)
        self.assertTrue(recipe.is_complete())
        self.assertFalse(recipe.is_faulty())


if __name__ == "__main__":
    unittest.main()
