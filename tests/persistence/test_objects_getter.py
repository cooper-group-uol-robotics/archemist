import unittest
from datetime import date
from archemist.core.util.location import Location
from archemist.core.state.material import Liquid, Solid
from archemist.core.state.robot import Robot, MobileRobot
from archemist.core.state.station import Station
from archemist.core.state.recipe import Recipe
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.persistence.objects_getter import (MaterialsGetter,
                                                       StationsGetter,
                                                       RobotsGetter,
                                                       LotsGetter,
                                                       BatchesGetter,
                                                       RecipesGetter)
from archemist.core.util.enums import LotStatus
from mongoengine import connect


class ObjectsGetterTest(unittest.TestCase):

    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        # construct materials
        liquid_dict = {
            'name': 'water',
            'id': 1254,
            'amount': 400,
            'unit': 'mL',
            'density': 997,
            'density_unit': 'kg/m3',
            'expiry_date': date.fromisoformat('2025-02-11'),
            'details': {
                'pump_id': 'pUmP1'
                }
        }

        solid_dict = {
            'name': 'sodium_chloride',
            'id': 133,
            'amount': 10000,
            'unit': 'ug',
            'expiry_date': date.fromisoformat('2025-02-11'),
            'details': {'dispense_src': 'quantos', 'cartridge_id': 123}
        }

        liquid = Liquid.from_dict(liquid_dict)
        solid = Solid.from_dict(solid_dict)

        # construct robots
        robot_dict_1 = {
            "type": "Robot",
            "location": {"node_id": 1, "graph_id": 2, "frame_name": "a_frame"},
            "id": 187,
            "handler": "SimRobotOpHandler"
        }

        robot_dict_2 = {
            "type": "MobileRobot",
            "location": {"node_id": 1, "graph_id": 2, "frame_name": "a_frame"},
            "id": 17,
            "batch_capacity":1,
            "handler": "SimRobotOpHandler"
        }

        self.robot_1 = Robot.from_dict(robot_dict_1)
        self.robot_2 = MobileRobot.from_dict(robot_dict_2)

        # construct stations
        station_dict_1 = {
            'type': 'Station',
            'id': 23,
            'location': {'node_id': 1, 'graph_id': 7},
            'handler': 'SimStationOpHandler',
            'total_lot_capacity': 2,
        }

        station_dict_2 = {
            'type': 'Station',
            'id': 24,
            'location': {'node_id': 2, 'graph_id': 7},
            'handler': 'SimStationOpHandler',
            'total_lot_capacity': 2,
        }

        self.station_1 = Station.from_dict(station_dict_1)
        self.station_2 = Station.from_dict(station_dict_2)
        
        # construct recipes
        recipe_doc_1 = {
            "general": {"name": "test_archemist_recipe", "id": 198},
            "steps": [
                {
                    "state_name": "start_state",
                    "station": {
                        "type": "Station",
                        "id": 23,
                        "process": {
                            "type": "StationProcess",
                            "operations": None,
                            "args": None,
                        },
                    },
                    "transitions": {
                        "on_success": "end_state",
                        "on_fail": "failed_state",
                    },
                },
            ],
        }

        recipe_doc_2 = {
            "general": {"name": "test_archemist_recipe", "id": 199},
            "steps": [
                {
                    "state_name": "start_state",
                    "station": {
                        "type": "Station",
                        "id": 23,
                        "process": {
                            "type": "StationProcess",
                            "operations": None,
                            "args": None,
                        },
                    },
                    "transitions": {
                        "on_success": "end_state",
                        "on_fail": "failed_state",
                    },
                },
            ],
        }

        recipe_1 = Recipe.from_dict(recipe_doc_1)
        recipe_2 = Recipe.from_dict(recipe_doc_2)

        # construct batches
        batch_1 = Batch.from_args(3, Location(1, 2, "some_frame"))
        batch_2 = Batch.from_args(3, Location(1, 2, "some_frame"))

        # construct lots
        lot_1 = Lot.from_args([batch_1])
        lot_2 = Lot.from_args([batch_2])

        # attach recipes
        lot_1.attach_recipe(recipe_1)
        lot_2.attach_recipe(recipe_2)

    def tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_materials_getter(self):
        liquids = MaterialsGetter.get_liquids()
        self.assertEqual(len(liquids), 1)
        self.assertEqual(liquids[0].id, 1254)

        solids = MaterialsGetter.get_solids()
        self.assertEqual(len(solids), 1)
        self.assertEqual(solids[0].id, 133)

    def test_stations_getter(self):
        # test getting all stations
        stations = StationsGetter.get_stations()
        self.assertEqual(len(stations), 2)
        self.assertEqual(stations[0].id, 23)
        self.assertEqual(stations[1].id, 24)

        # test getting individual station
        # by using object id
        station_by_obj_id = StationsGetter.get_station(self.station_1.object_id)
        self.assertEqual(station_by_obj_id.id, self.station_1.id)

        # by using station type and id
        station_by_type = StationsGetter.get_station("Station", 24)
        self.assertEqual(station_by_type.id, self.station_2.id)

    def test_robots_getter(self):
        # test getting all robots
        robots = RobotsGetter.get_robots()
        self.assertEqual(len(robots), 2)
        self.assertEqual(robots[0].id, 187)
        self.assertEqual(robots[1].id, 17)

        # get robots of specific types
        mobile_robots = RobotsGetter.get_robots("MobileRobot")
        self.assertEqual(len(mobile_robots), 1)
        self.assertEqual(mobile_robots[0].id, self.robot_2.id)
        
        # test getting individual robot
        # by using object id
        robot_by_obj_id = RobotsGetter.get_robot(self.robot_1.object_id)
        self.assertEqual(robot_by_obj_id.id, self.robot_1.id)

        # by using station type and id
        robot_by_type = RobotsGetter.get_robot("MobileRobot", 17)
        self.assertEqual(robot_by_type.id, self.robot_2.id)

    def test_lots_getter(self):
        lots = LotsGetter.get_lots()
        self.assertEqual(len(lots), 2)

        finished_lots = LotsGetter.get_finished_lots()
        self.assertFalse(finished_lots)
        lots[0].status = LotStatus.FINISHED
        finished_lots = LotsGetter.get_finished_lots()
        self.assertEqual(len(finished_lots), 1)
        self.assertEqual(finished_lots[0], lots[0])

    def test_batches_getter(self):
        batches = BatchesGetter.get_batches()
        self.assertEqual(len(batches), 2)

    def test_recipes_getter(self):
        recipes = RecipesGetter.get_recipes()
        self.assertEqual(len(recipes), 2)
        self.assertTrue(RecipesGetter.recipe_exists(198))
        self.assertFalse(RecipesGetter.recipe_exists(200))

if __name__ == '__main__':
    unittest.main()


