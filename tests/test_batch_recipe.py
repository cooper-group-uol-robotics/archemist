# External
import unittest
import yaml
from mongoengine import connect
import os
from unittest import TestCase

# Core
from archemist.stations.ika_digital_plate_station.state import (
    IKAHeatingOpDescriptor,
    IKAStirringOpDescriptor,
)
from archemist.core.state.batch import Batch
from archemist.core.state.recipe import Recipe
from archemist.core.util.location import Location


HERE = os.path.abspath(os.path.dirname(__file__))
RECIPE_FOLDER = os.path.join(HERE, 'resources')
RECIPE_FILE = os.path.join(RECIPE_FOLDER, 'testing_recipe.yaml')


def test_batch_from_dict():
    recipe_doc = []
    test_file = RECIPE_FILE
    with open(test_file) as testing_file:
        recipe_doc = yaml.safe_load(testing_file)

    batch = Batch.from_arguments(31, 2, Location(1, 3, "table_frame"))
    TestCase.assertEqual(batch.batch_id, 31)
    TestCase.assertEqual(batch.location, Location(1, 3, "table_frame"))
    batch.location = Location(1, 3, "chair_frame")
    TestCase.assertEqual(batch.location, Location(1, 3, "chair_frame"))
    TestCase.assertFalse(batch.recipe_attached)
    TestCase.assertIsNone(batch.recipe)

    TestCase.assertEqual(batch.num_samples, 2)
    TestCase.assertEqual(len(batch.station_history), 0)
    """ add recipe """
    recipe = Recipe.from_dict(recipe_doc)
    batch.attach_recipe(recipe)
    TestCase.assertTrue(batch.recipe_attached)
    TestCase.assertIsNotNone(batch.recipe)

    """ First operation """
    # process first sample
    TestCase.assertEqual(batch.current_sample_index, 0)
    ika_op1 = IKAHeatingOpDescriptor.from_args(temperature=50, duration=10)
    batch.add_station_op_to_current_sample(ika_op1)
    batch.add_material_to_current_sample("water")
    batch.process_current_sample()
    # process second sample
    TestCase.assertEqual(batch.current_sample_index, 1)
    batch.add_station_op_to_current_sample(ika_op1)
    batch.add_material_to_current_sample("water")
    batch.process_current_sample()
    batch.add_station_stamp("IkaRCTDigital-31")
    TestCase.assertEqual(len(batch.station_history), 1)
    
    """ Second operation """
    # process first sample
    TestCase.assertEqual(batch.current_sample_index, 0)
    ika_op2 = IKAStirringOpDescriptor.from_args(stirring_speed=50, duration=10)
    batch.add_station_op_to_current_sample(ika_op2)
    batch.add_material_to_current_sample("salt")
    batch.process_current_sample()
    # process second sample
    TestCase.assertEqual(batch.current_sample_index, 1)
    
    batch.add_station_op_to_current_sample(ika_op2)
    batch.add_material_to_current_sample("salt")
    batch.process_current_sample()
    batch.add_station_stamp("IkaRCTDigital-31")
    TestCase.assertEqual(len(batch.station_history), 2)

    samples = batch.get_samples_list()
    TestCase.assertEqual(len(samples), 2)
    TestCase.assertEqual(samples[0].rack_index, 0)
    TestCase.assertFalse(samples[0].capped)
    TestCase.assertEqual(len(samples[0].materials), 2)
    TestCase.assertEqual(samples[0].materials[0], "water")
    TestCase.assertEqual(len(samples[0].operation_ops), 2)
    TestCase.assertEqual(
        samples[0].operation_ops[0].target_temperature, ika_op1.target_temperature
    )
    TestCase.assertEqual(
        samples[0].operation_ops[0].target_duration, ika_op1.target_duration
    )

    # Test batch construction from objectId
    batch2 = Batch.from_object_id(batch._model.batch_id)
    TestCase.assertEqual(batch2.id, 31)
    TestCase.assertEqual(batch2.location, Location(1, 3, "chair_frame"))
    TestCase.assertEqual(len(batch2.station_history), 2)
    TestCase.assertEqual(batch2.recipe.current_state, "stirring_operation")
    TestCase.assertEqual(batch2.recipe.recipe_id, batch.recipe.recipe_id)
    TestCase.assertEqual(batch2.recipe.current_state, batch.recipe.current_state)


def test_recipe():
    recipe_doc = []
    with open(RECIPE_FILE) as testing_file:
        recipe_doc = yaml.safe_load(testing_file)

    batch = Batch.from_arguments(31, 2, Location(1, 3, "table_frame"))
    TestCase.assertFalse(batch.recipe_attached)
    TestCase.assertIsNone(batch.recipe)
    """add recipe"""
    recipe = Recipe.from_dict(recipe_doc)
    batch.attach_recipe(recipe)
    TestCase.assertTrue(batch.recipe_attached)

    TestCase.assertEqual(batch.recipe.recipe_id, 198)
    TestCase.assertEqual(batch.recipe.name, "test_archemist_recipe")
    TestCase.assertEqual(batch.recipe.liquids[0], {"name": "water", "id": 1})
    TestCase.assertEqual(batch.recipe.solids[0], {"name": "sodium_chloride", "id": 2})

    # stirring state
    TestCase.assertEqual(batch.recipe.current_state, "stirring_operation")
    
    station_name, station_id = batch.recipe.get_current_station()
    TestCase.assertEqual(station_name, "IkaPlateDigital")
    TestCase.assertEqual(station_id, 2)
    
    op1 = batch.recipe.get_current_task_op()
    TestCase.assertEqual(op1.__class__.__name__, "IKAStirringOpDescriptor")
    TestCase.assertEqual(op1.target_stirring_speed, 200)
    TestCase.assertEqual(op1.target_duration, 10)
    TestCase.assertFalse(batch.recipe.is_complete())

    # weighing state
    batch.recipe.advance_state(True)
    TestCase.assertEqual(batch.recipe.current_state, "weighing_operation")
    
    station_name, station_id = batch.recipe.get_current_station()
    TestCase.assertEqual(station_name, "FisherWeightingStation")
    TestCase.assertEqual(station_id, 5)
    
    op2 = batch.recipe.get_current_task_op()
    TestCase.assertEqual(op2.__class__.__name__, "FisherWeightOpDescriptor")
    TestCase.assertFalse(batch.recipe.is_complete())
    
    # end state
    batch.recipe.advance_state(True)
    TestCase.assertTrue(batch.recipe.is_complete())


if __name__ == "__main__":
    connect(
        db="archemist_test", host="mongodb://localhost:27017", alias="archemist_state"
    )
    unittest.main()
