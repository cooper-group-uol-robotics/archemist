import unittest

from archemist.persistence.persistenceManager import PersistenceManager

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


