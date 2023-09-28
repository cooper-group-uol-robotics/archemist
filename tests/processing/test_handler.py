import unittest
from mongoengine import connect
from archemist.core.state.robot import Robot
from archemist.core.state.station import Station
from archemist.core.state.robot_op import RobotOpDescriptor
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.processing.handler import RobotHandler, StationHandler, StationProcessHandler
from archemist.core.util.enums import RobotState, StationState, OpState, OpResult
from archemist.core.util.location import Location
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.state.recipe import Recipe
from archemist.core.state.station_process import StationProcess, ProcessStatus
from transitions import State

class TestProcess(StationProcess):

    def __init__(self, process_model) -> None:
        super().__init__(process_model)
        
        self.STATES = [State(name='init_state'),
            State(name='some_state'),
            State(name='final_state')]
        
        self.TRANSITIONS = [
            {'source':'init_state','dest':'some_state'},
            {'source':'some_state','dest':'final_state'}]

class HandlerTest(unittest.TestCase):
    def setUp(self) -> None:
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        self.robot_dict = {
            "type": "Robot",
            "location": {"node_id": 1, "graph_id": 2, "frame_name": "a_frame"},
            "id": 187,
            "batch_capacity":2,
            "handler": "SimRobotOpHandler"
        }

        self.station_dict = {
            'type': 'Station',
            'id': 23,
            'location': {'node_id': 1, 'graph_id': 7},
            'handler': 'SimStationOpHandler',
            'total_batch_capacity': 2,
            'process_batch_capacity': 1,
        }

        self.recipe_doc = {
            "general": {"name": "test_archemist_recipe", "id": 198},
            "steps": [
                {
                    "state_name": "start_state",
                    "station": {
                        "type": "Station",
                        "id": 2,
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

    def tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_robot_op_handler(self):
        robot = Robot.from_dict(self.robot_dict)
        self.assertEqual(robot.state, RobotState.INACTIVE)
        robot_handler = RobotHandler(robot, use_sim=False)
        robot_handler.initialise()
        self.assertEqual(robot.state, RobotState.ACTIVE)

        robot_op = RobotOpDescriptor.from_args()
        self.assertEqual(robot.assigned_op_state, OpState.INVALID)
        robot.add_op(robot_op)
        self.assertIsNotNone(robot.assigned_op)
        self.assertEqual(robot.assigned_op_state, OpState.ASSIGNED)
        self.assertIsNone(robot_op.start_timestamp)
        self.assertIsNone(robot_op.end_timestamp)

        robot_handler.tick()
        self.assertEqual(robot.assigned_op_state, OpState.EXECUTING)
        self.assertIsNotNone(robot_op.start_timestamp)

        robot_handler.tick()
        self.assertIsNone(robot.assigned_op)
        self.assertEqual(robot.assigned_op_state, OpState.INVALID)
        self.assertTrue(robot_op.has_result)
        self.assertEqual(robot_op.result, OpResult.SUCCEEDED)
        self.assertIsNotNone(robot_op.end_timestamp)

    def test_robot_op_handler_with_repeat(self):
        robot = Robot.from_dict(self.robot_dict)
        self.assertEqual(robot.state, RobotState.INACTIVE)
        robot_handler = RobotHandler(robot, use_sim=False)
        robot_handler.initialise()
        self.assertEqual(robot.state, RobotState.ACTIVE)

        robot_op = RobotOpDescriptor.from_args()
        self.assertEqual(robot.assigned_op_state, OpState.INVALID)
        robot.add_op(robot_op)
        self.assertIsNotNone(robot.assigned_op)
        self.assertEqual(robot.assigned_op_state, OpState.ASSIGNED)
        self.assertIsNone(robot_op.start_timestamp)
        self.assertIsNone(robot_op.end_timestamp)

        robot_handler.tick()
        self.assertEqual(robot.assigned_op_state, OpState.EXECUTING)
        time_stamp_1 = robot_op.start_timestamp
        self.assertIsNotNone(time_stamp_1)
        robot.repeat_assigned_op()
        self.assertEqual(robot.assigned_op_state, OpState.TO_BE_REPEATED)
        
        robot_handler.tick()
        self.assertEqual(robot.assigned_op_state, OpState.EXECUTING)
        time_stamp_2 = robot_op.start_timestamp
        self.assertIsNotNone(time_stamp_2)
        self.assertTrue(time_stamp_2 > time_stamp_1)

        robot_handler.tick()
        self.assertIsNone(robot.assigned_op)
        self.assertEqual(robot.assigned_op_state, OpState.INVALID)
        self.assertTrue(robot_op.has_result)
        self.assertEqual(robot_op.result, OpResult.SUCCEEDED)

    def test_robot_op_handler_with_skip(self):
        robot = Robot.from_dict(self.robot_dict)
        self.assertEqual(robot.state, RobotState.INACTIVE)
        robot_handler = RobotHandler(robot, use_sim=False)
        robot_handler.initialise()
        self.assertEqual(robot.state, RobotState.ACTIVE)

        robot_op = RobotOpDescriptor.from_args()
        self.assertEqual(robot.assigned_op_state, OpState.INVALID)
        robot.add_op(robot_op)
        self.assertIsNotNone(robot.assigned_op)
        self.assertEqual(robot.assigned_op_state, OpState.ASSIGNED)
        self.assertIsNone(robot_op.start_timestamp)
        self.assertIsNone(robot_op.end_timestamp)

        robot_handler.tick()
        self.assertEqual(robot.assigned_op_state, OpState.EXECUTING)
        self.assertIsNotNone(robot_op.start_timestamp)
        robot.skip_assigned_op()
        self.assertEqual(robot.assigned_op_state, OpState.TO_BE_SKIPPED)
        
        robot_handler.tick()
       
        self.assertIsNone(robot.assigned_op)
        self.assertEqual(robot.assigned_op_state, OpState.INVALID)
        self.assertTrue(robot_op.has_result)
        self.assertEqual(robot_op.result, OpResult.SKIPPED)

    def test_station_op_handler(self):
        station = Station.from_dict(self.station_dict)
        self.assertEqual(station.state, StationState.INACTIVE)
        station_handler = StationHandler(station, use_sim=False)
        station_handler.initialise()
        self.assertEqual(station.state, StationState.ACTIVE)

        station_op = StationOpDescriptor.from_args()
        station.add_station_op(station_op)
        self.assertIsNone(station.assigned_op)
        self.assertEqual(station.assigned_op_state, OpState.INVALID)
        
        station_handler.tick()
        self.assertEqual(station.assigned_op_state, OpState.ASSIGNED)
        self.assertIsNone(station_op.start_timestamp)
        self.assertIsNone(station_op.end_timestamp)

        station_handler.tick()
        self.assertEqual(station.assigned_op_state, OpState.EXECUTING)
        self.assertIsNotNone(station_op.start_timestamp)

        station_handler.tick()
        self.assertIsNone(station.assigned_op)
        self.assertEqual(station.assigned_op_state, OpState.INVALID)
        self.assertTrue(station_op.has_result)
        self.assertEqual(station_op.result, OpResult.SUCCEEDED)
        self.assertIsNotNone(station_op.end_timestamp)

    def test_station_op_handler_with_repeat(self):
        station = Station.from_dict(self.station_dict)
        self.assertEqual(station.state, StationState.INACTIVE)
        station_handler = StationHandler(station, use_sim=False)
        station_handler.initialise()
        self.assertEqual(station.state, StationState.ACTIVE)

        station_op = StationOpDescriptor.from_args()
        station.add_station_op(station_op)
        self.assertIsNone(station.assigned_op)
        self.assertEqual(station.assigned_op_state, OpState.INVALID)
        
        station_handler.tick()
        self.assertEqual(station.assigned_op_state, OpState.ASSIGNED)
        self.assertIsNone(station_op.start_timestamp)
        self.assertIsNone(station_op.end_timestamp)

        station_handler.tick()
        self.assertEqual(station.assigned_op_state, OpState.EXECUTING)
        time_stamp_1 = station_op.start_timestamp
        self.assertIsNotNone(time_stamp_1)
        station.repeat_assigned_op()
        self.assertEqual(station.assigned_op_state, OpState.TO_BE_REPEATED)
        self.assertIsNotNone(station_op.start_timestamp)

        station_handler.tick()
        self.assertEqual(station.assigned_op_state, OpState.EXECUTING)
        time_stamp_2 = station_op.start_timestamp
        self.assertIsNotNone(time_stamp_2)
        self.assertTrue(time_stamp_2 > time_stamp_1)

        station_handler.tick()
        self.assertIsNone(station.assigned_op)
        self.assertEqual(station.assigned_op_state, OpState.INVALID)
        self.assertTrue(station_op.has_result)
        self.assertEqual(station_op.result, OpResult.SUCCEEDED)
        self.assertIsNotNone(station_op.end_timestamp)

    def test_op_station_handler_with_skip(self):
        station = Station.from_dict(self.station_dict)
        self.assertEqual(station.state, StationState.INACTIVE)
        station_handler = StationHandler(station, use_sim=False)
        station_handler.initialise()
        self.assertEqual(station.state, StationState.ACTIVE)

        station_op = StationOpDescriptor.from_args()
        station.add_station_op(station_op)
        self.assertIsNone(station.assigned_op)
        self.assertEqual(station.assigned_op_state, OpState.INVALID)
        
        station_handler.tick()
        self.assertEqual(station.assigned_op_state, OpState.ASSIGNED)
        self.assertIsNone(station_op.start_timestamp)
        self.assertIsNone(station_op.end_timestamp)

        station_handler.tick()
        self.assertEqual(station.assigned_op_state, OpState.EXECUTING)
        self.assertIsNotNone(station_op.start_timestamp)
        station.skip_assigned_op()
        self.assertEqual(station.assigned_op_state, OpState.TO_BE_SKIPPED)

        station_handler.tick()
        self.assertIsNone(station.assigned_op)
        self.assertEqual(station.assigned_op_state, OpState.INVALID)
        self.assertTrue(station_op.has_result)
        self.assertEqual(station_op.result, OpResult.SKIPPED)
        self.assertIsNotNone(station_op.end_timestamp)

    def test_station_process_handler_with_added_procs(self):
        # construct lots
        batch_1 = Batch.from_args(3, Location(1, 2, "some_frame"))
        batch_2 = Batch.from_args(3, Location(1, 2, "some_frame"))
        lot_1 = Lot.from_args([batch_1])
        lot_2 = Lot.from_args([batch_2])
        
        # construct process
        proc_1 = TestProcess.from_args(lot_1)
        proc_2 = TestProcess.from_args(lot_2)

        # construct station and its process handler
        station = Station.from_dict(self.station_dict)
        proc_handler = StationProcessHandler(station)

        station_procs_dict = station.running_procs_slots
        self.assertEqual(len(station_procs_dict), 2)
        self.assertFalse(station.queued_procs)

        # add procs to station
        station.add_process(proc_1)
        station.add_process(proc_2)
        self.assertEqual(len(station.queued_procs), 2)
        self.assertIsNone(station_procs_dict["0"])
        self.assertIsNone(station_procs_dict["1"])
        
        # test processes assignment
        proc_handler.handle()
        self.assertEqual(station_procs_dict["0"].uuid, proc_1.uuid)
        self.assertEqual(station_procs_dict["1"].uuid, proc_2.uuid)
        self.assertEqual(len(station.queued_procs), 0)

        # test processes state advancement
        proc_handler.handle()
        self.assertEqual(station_procs_dict["0"].m_state, "some_state")
        self.assertEqual(station_procs_dict["1"].m_state, "some_state")
        self.assertEqual(station_procs_dict["0"].status, ProcessStatus.RUNNING)
        self.assertEqual(station_procs_dict["1"].status, ProcessStatus.RUNNING)

        # test processes state advancement
        proc_handler.handle()
        self.assertEqual(station_procs_dict["0"].m_state, "final_state")
        self.assertEqual(station_procs_dict["1"].m_state, "final_state")
        self.assertEqual(station_procs_dict["0"].status, ProcessStatus.FINISHED)
        self.assertEqual(station_procs_dict["1"].status, ProcessStatus.FINISHED)

         # test processes completion
        proc_handler.handle()
        self.assertIsNone(station_procs_dict["0"])
        self.assertIsNone(station_procs_dict["1"])
        self.assertEqual(proc_1.status, ProcessStatus.FINISHED)
        self.assertEqual(proc_2.status, ProcessStatus.FINISHED)
        self.assertEqual(len(station.procs_history), 2)

    def test_station_process_handler_with_added_lots(self):
        # construct lots and their recipes
        batch_1 = Batch.from_args(3, Location(1, 2, "some_frame"))
        batch_2 = Batch.from_args(3, Location(1, 2, "some_frame"))
        lot_1 = Lot.from_args([batch_1])
        lot_2 = Lot.from_args([batch_2])
        recipe_1_doc = self.recipe_doc
        recipe_1 = Recipe.from_dict(recipe_1_doc)
        lot_1.attach_recipe(recipe_1)

        recipe_2_doc = self.recipe_doc
        recipe_2_doc["general"]["id"] = 199
        recipe_2 = Recipe.from_dict(recipe_2_doc)
        lot_2.attach_recipe(recipe_2)

        # construct station and its process handler
        station = Station.from_dict(self.station_dict)
        proc_handler = StationProcessHandler(station)

        station_procs_dict = station.running_procs_slots
        self.assertEqual(len(station_procs_dict), 2)
        self.assertFalse(station.queued_procs)

        # add procs to station
        station.add_lot(lot_1)
        station.add_lot(lot_2)
       
        self.assertFalse(station.queued_procs)
        self.assertIsNone(station_procs_dict["0"])
        self.assertIsNone(station_procs_dict["1"])
        
        # test processes assignment
        proc_handler.handle()
        self.assertIsNotNone(station_procs_dict["0"].uuid)
        self.assertIsNotNone(station_procs_dict["1"].uuid)
        self.assertEqual(station_procs_dict["0"].status, ProcessStatus.INACTIVE)
        self.assertEqual(station_procs_dict["1"].status, ProcessStatus.INACTIVE)

        # test processes state advancement
        proc_handler.handle()
        self.assertEqual(station_procs_dict["0"].m_state, "final_state")
        self.assertEqual(station_procs_dict["1"].m_state, "final_state")
        self.assertEqual(station_procs_dict["0"].status, ProcessStatus.FINISHED)
        self.assertEqual(station_procs_dict["1"].status, ProcessStatus.FINISHED)

        # test processes completion
        proc_handler.handle()
        self.assertIsNone(station_procs_dict["0"])
        self.assertIsNone(station_procs_dict["1"])
        self.assertEqual(len(station.procs_history), 2)

    def test_station_process_handler_mixed(self):
        # construct lots and their recipes
        batch_1 = Batch.from_args(3, Location(1, 2, "some_frame"))
        batch_2 = Batch.from_args(3, Location(1, 2, "some_frame"))
        lot_1 = Lot.from_args([batch_1])
        lot_2 = Lot.from_args([batch_2])
        recipe_1_doc = self.recipe_doc
        recipe_1 = Recipe.from_dict(recipe_1_doc)
        lot_1.attach_recipe(recipe_1)

        recipe_2_doc = self.recipe_doc
        recipe_2_doc["general"]["id"] = 199
        recipe_2 = Recipe.from_dict(recipe_2_doc)
        lot_2.attach_recipe(recipe_2)

        # construct process
        batch_3 = Batch.from_args(3, Location(1, 2, "some_frame"))
        lot_3 = Lot.from_args([batch_3])
        proc = TestProcess.from_args(lot_3)

        # construct station and its process handler
        station = Station.from_dict(self.station_dict)
        proc_handler = StationProcessHandler(station)

        station_procs_dict = station.running_procs_slots
        self.assertEqual(len(station_procs_dict), 2)
        self.assertFalse(station.queued_procs)

        # add lots and procs to station
        station.add_lot(lot_1)
        station.add_process(proc)
       
        self.assertEqual(len(station.queued_procs), 1)
        self.assertIsNone(station_procs_dict["0"])
        self.assertIsNone(station_procs_dict["1"])
        
        # test processes assignment
        proc_handler.handle()
        self.assertEqual(len(station.queued_procs), 0)
        self.assertEqual(station_procs_dict["0"].uuid, proc.uuid)
        self.assertIsNotNone(station_procs_dict["1"].uuid)
        self.assertEqual(station_procs_dict["0"].status, ProcessStatus.INACTIVE)
        self.assertEqual(station_procs_dict["1"].status, ProcessStatus.INACTIVE)

        # add a lot
        station.add_lot(lot_2)

        # test processes state advancement
        proc_handler.handle()
        self.assertEqual(len(station.queued_procs), 1)
        self.assertEqual(station_procs_dict["0"].m_state, "some_state")
        self.assertEqual(station_procs_dict["1"].m_state, "final_state")
        self.assertEqual(station_procs_dict["0"].status, ProcessStatus.RUNNING)
        self.assertEqual(station_procs_dict["1"].status, ProcessStatus.FINISHED)

        # test processes state advancement
        proc_handler.handle()
        self.assertEqual(station_procs_dict["0"].m_state, "final_state")
        self.assertEqual(station_procs_dict["0"].status, ProcessStatus.FINISHED)
        self.assertIsNone(station_procs_dict["1"])
        self.assertEqual(len(station.procs_history), 1)

        # test processes state advancement
        proc_handler.handle()
        self.assertIsNone(station_procs_dict["0"])
        self.assertEqual(proc.status, ProcessStatus.FINISHED)
        self.assertIsNotNone(station_procs_dict["1"].uuid)
        self.assertEqual(station_procs_dict["1"].status, ProcessStatus.INACTIVE)
        self.assertEqual(len(station.procs_history), 2)

        # test processes state advancement
        proc_handler.handle()
        self.assertIsNone(station_procs_dict["0"])
        self.assertEqual(station_procs_dict["1"].m_state, "final_state")
        self.assertEqual(station_procs_dict["1"].status, ProcessStatus.FINISHED)

        # test processes completion
        proc_handler.handle()
        self.assertFalse(station.queued_procs)
        self.assertIsNone(station_procs_dict["0"])
        self.assertIsNone(station_procs_dict["1"])
        self.assertEqual(len(station.procs_history), 3)

        # test again processes completion
        proc_handler.handle()
        self.assertFalse(station.queued_procs)
        self.assertIsNone(station_procs_dict["0"])
        self.assertIsNone(station_procs_dict["1"])
        self.assertEqual(len(station.procs_history), 3)

if __name__ == "__main__":
    unittest.main()
        