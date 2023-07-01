import unittest
from archemist.stations.ika_digital_plate_station.state import IKAHeatingOpDescriptor, IKAStirringOpDescriptor
from archemist.stations.fisher_balance_station.state import FisherWeightOpDescriptor
from archemist.core.state.recipe import Recipe
import yaml
from pathlib import Path
from mongoengine import connect

class RecipeTest(unittest.TestCase):
     def setUp(self):
         connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')

     def test_recipe(self):
        recipe_doc = dict()
        resources_path = Path.joinpath(Path.cwd(), "tests/state/resources/testing_recipe.yaml")
        with resources_path.open() as fs:
            recipe_doc = yaml.load(fs, Loader=yaml.SafeLoader)
        
        '''create recipe'''
        recipe = Recipe.from_dict(recipe_doc)
        
        self.assertEqual(recipe.id, 198)
        self.assertEqual(recipe.name, 'test_archemist_recipe')
        self.assertEqual(recipe.liquids[0], {'name':'water', 'id':1})
        self.assertEqual(recipe.solids[0], {'name':'sodium_chloride', 'id':2})

        # stirring state
        self.assertEqual(recipe.current_state, 'stirring_operation')
        station_name, station_id = recipe.get_current_station()
        self.assertEqual(station_name, 'IkaPlateDigital')
        self.assertEqual(station_id, 2)
        process_1 = recipe.get_current_process()
        self.assertEqual(process_1["type"], "CrystalBotWorkflowProcess")
        self.assertIsNone(process_1['args'])
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
        self.assertEqual(recipe.current_state, 'weighing_operation')
        station_name, station_id = recipe.get_current_station()
        self.assertEqual(station_name, 'FisherWeightingStation')
        self.assertEqual(station_id, 5)
        process_2 = recipe.get_current_process()
        self.assertEqual(process_2["type"], "SomeProcess")
        self.assertEqual(process_2['args'], {"some_variable": 42})
        op_2 = recipe.get_current_task_op()
        self.assertTrue(isinstance(op_2, FisherWeightOpDescriptor))
        self.assertFalse(recipe.is_complete())
        next_station, next_station_id = recipe.get_next_station(success=True)
        self.assertEqual(next_station, "end")
        self.assertIsNone(next_station_id)
        # end state
        recipe.advance_state(True)
        self.assertTrue(recipe.is_complete())

if __name__ == '__main__':
    unittest.main()