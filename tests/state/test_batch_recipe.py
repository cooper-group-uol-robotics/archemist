import unittest
from archemist.stations.ika_digital_plate_station.state import (
    IKAHeatingOpDescriptor,
    IKAStirringOpDescriptor,
)
from archemist.core.state.batch import Batch
from archemist.core.state.recipe import Recipe
import yaml
from mongoengine import connect
from archemist.core.util.location import Location


class BatchRecipeTest(unittest.TestCase):
    def setUp(self):
        self.batch_obj_id = None

    def test_batch_from_dict(self):
        recipe_doc = dict()
        with open("resources/testing_recipe.yaml") as fs:
            recipe_doc = yaml.load(fs, Loader=yaml.SafeLoader)

        batch = Batch.from_arguments(31, 2, Location(1, 3, "table_frame"))
        self.assertEqual(batch.id, 31)
        self.assertEqual(batch.location, Location(1, 3, "table_frame"))
        batch.location = Location(1, 3, "chair_frame")
        self.assertEqual(batch.location, Location(1, 3, "chair_frame"))
        self.assertFalse(batch.recipe_attached)
        self.assertIsNone(batch.recipe)

        self.assertEqual(batch.num_samples, 2)
        self.assertEqual(len(batch.station_history), 0)
        """ add recipe """
        recipe = Recipe.from_dict(recipe_doc)
        batch.attach_recipe(recipe)
        self.assertTrue(batch.recipe_attached)
        self.assertIsNotNone(batch.recipe)

        """ First operation """
        # process first sample
        self.assertEqual(batch.current_sample_index, 0)
        ika_op1 = IKAHeatingOpDescriptor.from_args(temperature=50, duration=10)
        batch.add_station_op_to_current_sample(ika_op1)
        batch.add_material_to_current_sample("water")
        batch.process_current_sample()
        # process second sample
        self.assertEqual(batch.current_sample_index, 1)
        batch.add_station_op_to_current_sample(ika_op1)
        batch.add_material_to_current_sample("water")
        batch.process_current_sample()
        batch.add_station_stamp("IkaRCTDigital-31")
        self.assertEqual(len(batch.station_history), 1)
        """ Second operation """
        # process first sample
        self.assertEqual(batch.current_sample_index, 0)
        ika_op2 = IKAStirringOpDescriptor.from_args(stirring_speed=50, duration=10)
        batch.add_station_op_to_current_sample(ika_op2)
        batch.add_material_to_current_sample("salt")
        batch.process_current_sample()
        # process second sample
        self.assertEqual(batch.current_sample_index, 1)
        batch.add_station_op_to_current_sample(ika_op2)
        batch.add_material_to_current_sample("salt")
        batch.process_current_sample()
        batch.add_station_stamp("IkaRCTDigital-31")
        self.assertEqual(len(batch.station_history), 2)

        samples = batch.get_samples_list()
        self.assertEqual(len(samples), 2)
        self.assertEqual(samples[0].rack_index, 0)
        self.assertFalse(samples[0].capped)
        self.assertEqual(len(samples[0].materials), 2)
        self.assertEqual(samples[0].materials[0], "water")
        self.assertEqual(len(samples[0].operation_ops), 2)
        self.assertEqual(
            samples[0].operation_ops[0].target_temperature, ika_op1.target_temperature
        )
        self.assertEqual(
            samples[0].operation_ops[0].target_duration, ika_op1.target_duration
        )

        # Test batch construction from objectId
        batch2 = Batch.from_object_id(batch._model.id)
        self.assertEqual(batch2.id, 31)
        self.assertEqual(batch2.location, Location(1, 3, "chair_frame"))
        self.assertEqual(len(batch2.station_history), 2)
        self.assertEqual(batch2.recipe.current_state, "stirring_operation")

        self.assertEqual(batch2.recipe.id, batch.recipe.id)
        self.assertEqual(batch2.recipe.current_state, batch.recipe.current_state)

    def test_recipe(self):
        recipe_doc = dict()
        with open("resources/testing_recipe.yaml") as fs:
            recipe_doc = yaml.load(fs, Loader=yaml.SafeLoader)

        batch = Batch.from_arguments(31, 2, Location(1, 3, "table_frame"))
        self.assertFalse(batch.recipe_attached)
        self.assertIsNone(batch.recipe)
        """add recipe"""
        recipe = Recipe.from_dict(recipe_doc)
        batch.attach_recipe(recipe)
        self.assertTrue(batch.recipe_attached)

        self.assertEqual(batch.recipe.id, 198)
        self.assertEqual(batch.recipe.name, "test_archemist_recipe")
        self.assertEqual(batch.recipe.liquids[0], {"name": "water", "id": 1})
        self.assertEqual(batch.recipe.solids[0], {"name": "sodium_chloride", "id": 2})

        # stirring state
        self.assertEqual(batch.recipe.current_state, "stirring_operation")
        station_name, station_id = batch.recipe.get_current_station()
        self.assertEqual(station_name, "IkaPlateDigital")
        self.assertEqual(station_id, 2)
        op1 = batch.recipe.get_current_task_op()
        self.assertEqual(op1.__class__.__name__, "IKAStirringOpDescriptor")
        self.assertEqual(op1.target_stirring_speed, 200)
        self.assertEqual(op1.target_duration, 10)
        self.assertFalse(batch.recipe.is_complete())

        # weighing state
        batch.recipe.advance_state(True)
        self.assertEqual(batch.recipe.current_state, "weighing_operation")
        station_name, station_id = batch.recipe.get_current_station()
        self.assertEqual(station_name, "FisherWeightingStation")
        self.assertEqual(station_id, 5)
        op2 = batch.recipe.get_current_task_op()
        self.assertEqual(op2.__class__.__name__, "FisherWeightOpDescriptor")
        self.assertFalse(batch.recipe.is_complete())
        # end state
        batch.recipe.advance_state(True)
        self.assertTrue(batch.recipe.is_complete())


if __name__ == "__main__":
    connect(
        db="archemist_test", host="mongodb://localhost:27017", alias="archemist_state"
    )
    unittest.main()
