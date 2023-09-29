import unittest
from datetime import date
from archemist.core.util.location import Location
from archemist.core.state.material import Liquid, Solid
from archemist.core.state.robot import Robot, MobileRobot
from archemist.core.state.robot_op import RobotOpDescriptor
from archemist.core.state.station import Station
from archemist.core.state.station_process import StationProcess
from archemist.core.state.recipe import Recipe
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.state.state import State
from mongoengine import connect


class StateTest(unittest.TestCase):

    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        self.state_dict = {
            "name": "test_workflow",
        }

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
        self.recipe_doc_1 = {
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

        self.recipe_doc_3 = {
            "general": {"name": "test_archemist_recipe", "id": 200},
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

        recipe_1 = Recipe.from_dict(self.recipe_doc_1)
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

    def test_state_fields(self):
        state = State.from_dict(self.state_dict)
        self.assertEqual(state.workflow_name, "test_workflow")

        # test material fields
        liquids = state.liquids
        self.assertEqual(len(liquids), 1)
        self.assertEqual(liquids[0].id, 1254)

        solids = state.solids
        self.assertEqual(len(solids), 1)
        self.assertEqual(solids[0].id, 133)

        # test stations fields
        stations = state.stations
        self.assertEqual(len(stations), 2)
        self.assertEqual(stations[0].id, 23)
        self.assertEqual(stations[1].id, 24)

        # test robots fields
        robots = state.robots
        self.assertEqual(len(robots), 2)
        self.assertEqual(robots[0].id, 187)
        self.assertEqual(robots[1].id, 17)

        # test recipes fields
        recipes = state.recipes
        self.assertEqual(len(recipes), 2)
        
        self.assertFalse(state.recipes_queue)
        state.recipes_queue.append(recipes[0])
        self.assertEqual(len(state.recipes_queue), 1)
        self.assertIsNotNone(state.recipes_queue.pop())
        self.assertFalse(state.recipes_queue)

        self.assertFalse(state.is_new_recipe_dict(self.recipe_doc_1))
        self.assertTrue(state.is_new_recipe_dict(self.recipe_doc_3))

        # test lots fields
        lots = state.lots
        self.assertEqual(len(lots), 2)
        self.assertFalse(state.lots_buffer)
        state.lots_buffer.append(lots[0])
        self.assertEqual(len(state.lots_buffer), 1)
        self.assertIsNotNone(state.lots_buffer.pop())
        self.assertFalse(state.lots_buffer)

        completed_lots = state.get_completed_lots()
        self.assertFalse(completed_lots)
        recipes[0].advance_state(True)
        completed_lots = state.get_completed_lots()
        self.assertEqual(len(completed_lots), 1)
        self.assertEqual(completed_lots[0], lots[0])

        # test robot op queue
        self.assertFalse(state.robot_ops_queue)
        robot_op = RobotOpDescriptor.from_args()
        state.robot_ops_queue.append(robot_op)
        self.assertEqual(len(state.robot_ops_queue), 1)
        self.assertIsNotNone(state.robot_ops_queue.pop())
        self.assertFalse(state.robot_ops_queue)

        # test station process
        self.assertFalse(state.proc_requests_queue)
        station_proc = StationProcess.from_args(lots[0])
        state.proc_requests_queue.append(station_proc)
        self.assertEqual(len(state.proc_requests_queue), 1)
        self.assertIsNotNone(state.proc_requests_queue.pop())
        self.assertFalse(state.proc_requests_queue)

        # test getting individual station
        # by using object id
        station_by_obj_id = state.get_station(self.station_1.object_id)
        self.assertEqual(station_by_obj_id.id, self.station_1.id)

        # by using station type and id
        station_by_type = state.get_station("Station", 24)
        self.assertEqual(station_by_type.id, self.station_2.id)

        # test getting individual robot
        # by using object id
        robot_by_obj_id = state.get_robot(self.robot_1.object_id)
        self.assertEqual(robot_by_obj_id.id, self.robot_1.id)

        # by using station type and id
        robot_by_type = state.get_robot("MobileRobot", 17)
        self.assertEqual(robot_by_type.id, self.robot_2.id)

        # get robots of specific types
        mobile_robots = state.get_robots_of_type("MobileRobot")
        self.assertEqual(len(mobile_robots), 1)
        self.assertEqual(mobile_robots[0].id, self.robot_2.id)


if __name__ == '__main__':
    unittest.main()


