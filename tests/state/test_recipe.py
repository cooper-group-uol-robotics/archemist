import unittest
from archemist.core.state.recipe import Recipe
from mongoengine import connect


class RecipeTest(unittest.TestCase):
    def setUp(self) -> None:
        self._db_name = "archemist_test"
        self._client = connect(
            db=self._db_name, host="mongodb://localhost:27017", alias="archemist_state"
        )

        self._recipe_doc = {
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

    def tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_recipe(self):
        # construct a recipe
        recipe = Recipe.from_dict(self._recipe_doc)

        self.assertEqual(recipe.id, 198)
        self.assertEqual(recipe.name, "test_archemist_recipe")

        # stirring state
        self.assertEqual(recipe.current_state, "stirring_operation")
        current_state_details = recipe.current_state_details
        self.assertEqual(current_state_details.station_type, "IkaPlateDigital")
        self.assertEqual(current_state_details.station_id, 2)
        process_1 = current_state_details.station_process
        self.assertEqual(process_1["type"], "CrystalBotWorkflowProcess")
        self.assertIsNone(process_1["args"])
        key_operations_1 = [
            {   "name" : "stir",
                "type": "IKAStirringOpDescriptor",
                "repeat_for_all_batches": False,
                "parameters": [
                    {"stirring_speed": 200, "duration": 10},
                    {"stirring_speed": 150, "duration": 5}
                    ],
            },
        ]
        self.assertListEqual(process_1["operations"], key_operations_1)
        self.assertFalse(recipe.is_complete())
        self.assertFalse(recipe.is_failed())
        next_state_details = recipe.get_next_state_details(success=True)
        self.assertEqual(next_state_details.station_type, "FisherWeightingStation")
        self.assertEqual(next_state_details.station_id, 5)
        process_2 = next_state_details.station_process
        self.assertEqual(process_2["type"], "SomeProcess")

        # weighing state
        recipe.advance_state(True)
        self.assertEqual(recipe.current_state, "weighing_operation")
        current_state_details = recipe.current_state_details
        self.assertEqual(current_state_details.station_type, "FisherWeightingStation")
        self.assertEqual(current_state_details.station_id, 5)
        process_2 = current_state_details.station_process
        self.assertEqual(process_2["type"], "SomeProcess")
        self.assertEqual(process_2["args"], {"some_variable": 42})
        key_operations_2 = [
            {
                "name": "weigh",
                "type": "FisherWeightOpDescriptor",
                "repeat_for_all_batches": True,
                "parameters": [{"some_param": 123}],
            },
        ]
        self.assertListEqual(process_2["operations"], key_operations_2)
        self.assertFalse(recipe.is_complete())

        self.assertIsNone(recipe.get_next_state_details(success=True))
        # end state
        recipe.advance_state(True)
        self.assertTrue(recipe.is_complete())
        self.assertFalse(recipe.is_failed())

    def test_recipe_failed(self):
        # construct a recipe
        recipe = Recipe.from_dict(self._recipe_doc)
        self.assertFalse(recipe.is_complete())
        self.assertFalse(recipe.is_failed())

        # advance to failed state
        recipe.advance_state(False)

        self.assertFalse(recipe.is_complete())
        self.assertTrue(recipe.is_failed())


if __name__ == "__main__":
    unittest.main()
