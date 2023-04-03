import unittest
from archemist.core.persistence.persistence_manager import PersistenceManager
from archemist.core.util.location import Location
from archemist.core.state.recipe import Recipe
import yaml
import os


class StateTest(unittest.TestCase):
    def test_state_config(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        config_dir = os.path.join(dir_path, "resources/testing_config_file.yaml")
        host = "mongodb://localhost:27017"
        pm = PersistenceManager(host, "archemist_test")
        state = pm.construct_state_from_config_file(config_dir)

        stations = state.stations
        self.assertEqual(len(stations), 4)
        self.assertEqual(stations[0].__class__.__name__, "FisherWeightingStation")
        self.assertEqual(stations[0].id, 5)
        obj_id_station = state.get_station(stations[0]._model.id)
        self.assertEqual(obj_id_station.__class__.__name__, "FisherWeightingStation")
        self.assertEqual(obj_id_station.id, 5)
        string_id_station = state.get_station("FisherWeightingStation", 5)
        self.assertEqual(string_id_station.__class__.__name__, "FisherWeightingStation")
        self.assertEqual(string_id_station.id, 5)
        string_id_station = state.get_station("ChemSpeedFlexStation", 9)
        self.assertEqual(stations[1].__class__.__name__, "ChemSpeedFlexStation")
        self.assertEqual(stations[1].id, 9)
        self.assertEqual(stations[2].__class__.__name__, "IkaPlateDigital")
        self.assertEqual(stations[2].id, 2)

        robots = state.robots
        self.assertEqual(len(robots), 2)
        self.assertEqual(robots[0].__class__.__name__, "PandaFranka")
        self.assertEqual(robots[0].id, 99)
        obj_id_robot = state.get_robot(robots[0]._model.id)
        self.assertEqual(obj_id_robot.__class__.__name__, "PandaFranka")
        self.assertEqual(obj_id_robot.id, 99)
        string_id_robot = state.get_robot("PandaFranka", 99)
        self.assertEqual(string_id_robot.__class__.__name__, "PandaFranka")
        self.assertEqual(string_id_robot.id, 99)

        liquids = state.liquids
        self.assertEqual(len(liquids), 1)
        self.assertEqual(liquids[0].name, "water")
        self.assertEqual(liquids[0].id, 145)

        solids = state.solids
        self.assertEqual(len(solids), 1)
        self.assertEqual(solids[0].name, "sodium_chloride")
        self.assertEqual(solids[0].id, 345)

        self.assertEqual(len(state.batches), 0)
        recipe_doc = dict()
        recipe_dir = os.path.join(dir_path, "resources/testing_recipe.yaml")
        with open(recipe_dir) as fs:
            recipe_doc = yaml.load(fs, Loader=yaml.SafeLoader)
        batch1 = state.add_clean_batch()
        batch2 = state.add_clean_batch()

        batches = state.batches
        self.assertEqual(len(batches), 2)

        clean_batches = state.get_clean_batches()
        self.assertEqual(len(clean_batches), 2)
        recipe = Recipe.from_dict(recipe_doc)
        batch1.attach_recipe(recipe)

        clean_batches = state.get_clean_batches()
        self.assertEqual(len(clean_batches), 1)

        batch1.recipe.advance_state(True)
        batch1.recipe.advance_state(True)

        obj_id_batch = state.get_batch(batch1._model.id)
        self.assertEqual(obj_id_batch.id, batch1.id)
        self.assertTrue(obj_id_batch.recipe.is_complete())

        only_id_batch = state.get_batch(1)
        self.assertFalse(only_id_batch.recipe_attached)

        processed_batches = state.get_completed_batches()
        self.assertEqual(len(processed_batches), 1)

        batches = state.batches
        self.assertEqual(len(batches), 2)

    def test_state_db(self):
        host = "mongodb://localhost:27017"
        pm = PersistenceManager(host, "archemist_test")
        state = pm.construct_state_from_db()

        stations = state.stations
        self.assertEqual(len(stations), 4)
        self.assertEqual(stations[0].__class__.__name__, "FisherWeightingStation")
        self.assertEqual(stations[0].id, 5)
        self.assertEqual(stations[1].__class__.__name__, "ChemSpeedFlexStation")
        self.assertEqual(stations[1].id, 9)
        self.assertEqual(stations[2].__class__.__name__, "IkaPlateDigital")
        self.assertEqual(stations[2].id, 2)
        string_id_station = state.get_station("ChemSpeedFlexStation", 9)
        self.assertEqual(string_id_station.__class__.__name__, "ChemSpeedFlexStation")
        self.assertEqual(string_id_station.id, 9)

        robots = state.robots
        self.assertEqual(len(robots), 2)
        self.assertEqual(robots[0].__class__.__name__, "PandaFranka")
        self.assertEqual(robots[0].id, 99)

        liquids = state.liquids
        self.assertEqual(len(liquids), 1)
        self.assertEqual(liquids[0].name, "water")
        self.assertEqual(liquids[0].id, 145)

        solids = state.solids
        self.assertEqual(len(solids), 1)
        self.assertEqual(solids[0].name, "sodium_chloride")
        self.assertEqual(solids[0].id, 345)


if __name__ == "__main__":
    unittest.main()
