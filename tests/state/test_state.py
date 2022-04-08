import unittest

from archemist.persistence.persistenceManager import PersistenceManager
from archemist.state.batch import Batch
from archemist.util.location import Location
import yaml
import os

class StateTest(unittest.TestCase):

    def test_state_config(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        config_dir = os.path.join(dir_path, 'resources/testing_config_file.yaml')
        pm = PersistenceManager('test')
        state = pm.construct_state_from_config_file(config_dir)

        stations = state.stations
        self.assertEqual(len(stations), 3)
        self.assertEqual(stations[0].__class__.__name__, 'PeristalticLiquidDispensing')
        self.assertEqual(stations[0].id, 23)
        obj_id_station = state.get_station(stations[0].object_id)
        self.assertEqual(obj_id_station.__class__.__name__, 'PeristalticLiquidDispensing')
        self.assertEqual(obj_id_station.id, 23)
        string_id_station = state.get_station('PeristalticLiquidDispensing', 23)
        self.assertEqual(string_id_station.__class__.__name__, 'PeristalticLiquidDispensing')
        self.assertEqual(string_id_station.id, 23)
        self.assertEqual(stations[1].__class__.__name__, 'IkaPlateRCTDigital')
        self.assertEqual(stations[1].id, 2)
        self.assertEqual(stations[2].__class__.__name__, 'FisherWeightingStation')
        self.assertEqual(stations[2].id, 5)

        robots = state.robots
        self.assertEqual(len(robots), 1)
        self.assertEqual(robots[0].__class__.__name__, 'PandaFranka')
        self.assertEqual(robots[0].id, 1)
        obj_id_robot = state.get_station(robots[0].object_id)
        self.assertEqual(obj_id_robot.__class__.__name__, 'PandaFranka')
        self.assertEqual(obj_id_robot.id, 1)
        string_id_robot = state.get_station('PandaFranka', 1)
        self.assertEqual(string_id_robot.__class__.__name__, 'PandaFranka')
        self.assertEqual(string_id_robot.id, 1)

        liquids = state.liquids
        self.assertEqual(len(liquids), 1)
        self.assertEqual(liquids[0].name, 'water')
        self.assertEqual(liquids[0].id, 145)

        solids = state.solids
        self.assertEqual(len(solids), 1)
        self.assertEqual(solids[0].name, 'sodium_chloride')
        self.assertEqual(solids[0].id, 345)

        self.assertEqual(len(state.batches), 0)
        recipe_doc = dict()
        recipe_dir = os.path.join(dir_path, 'resources/testing_recipe.yaml')
        with open(recipe_dir) as fs:
            recipe_doc = yaml.load(fs, Loader=yaml.SafeLoader)
        batch1 = Batch.from_arguments('test',31,recipe_doc,2,Location(1,3,'table_frame'))
        batch2 = Batch.from_arguments('test',77,recipe_doc,2,Location(1,3,'table_frame'))

        batches = state.batches
        self.assertEqual(len(batches), 2)

        batch1.recipe.advance_state(True)
        batch1.recipe.advance_state(True)
        batch1.recipe.advance_state(True)
        
        processed_batches = state.completed_batches
        self.assertEqual(len(processed_batches), 1)

    def test_state_db(self):
        pm = PersistenceManager('test')
        state = pm.construct_state_from_db()

        stations = state.stations
        self.assertEqual(len(stations), 3)
        self.assertEqual(stations[0].__class__.__name__, 'PeristalticLiquidDispensing')
        self.assertEqual(stations[0].id, 23)
        self.assertEqual(stations[1].__class__.__name__, 'IkaPlateRCTDigital')
        self.assertEqual(stations[1].id, 2)
        self.assertEqual(stations[2].__class__.__name__, 'FisherWeightingStation')
        self.assertEqual(stations[2].id, 5)

        robots = state.robots
        self.assertEqual(len(robots), 1)
        self.assertEqual(robots[0].__class__.__name__, 'PandaFranka')
        self.assertEqual(robots[0].id, 1)

        liquids = state.liquids
        self.assertEqual(len(liquids), 1)
        self.assertEqual(liquids[0].name, 'water')
        self.assertEqual(liquids[0].id, 145)

        solids = state.solids
        self.assertEqual(len(solids), 1)
        self.assertEqual(solids[0].name, 'sodium_chloride')
        self.assertEqual(solids[0].id, 345)


if __name__ == '__main__':
    unittest.main()


