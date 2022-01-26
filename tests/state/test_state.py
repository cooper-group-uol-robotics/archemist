import unittest

from archemist.persistence.persistenceManager import PersistenceManager
from archemist.state.batch import Batch
from archemist.util.location import Location
import yaml

class StateTest(unittest.TestCase):

    def test_state_config(self):
        pm = PersistenceManager('test')
        state = pm.construct_state_from_config_file('/home/gilgamish/robot_chemist_ws/src/archemist/tests/state/resources/testing_config_file.yaml')

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

        self.assertEqual(len(state.batches), 0)
        recipe_doc = dict()
        with open('/home/gilgamish/robot_chemist_ws/src/archemist/tests/state/resources/testing_recipe.yaml') as fs:
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


