import unittest
from time import sleep
from mongoengine import connect
from archemist.core.state.robot import Robot
from archemist.core.state.station import Station
from archemist.core.state.robot_op import RobotOpDescriptor, RobotWaitOpDescriptor
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.processing.handler import RobotHandler, StationHandler, StationProcessHandler
from archemist.core.util.enums import RobotState, StationState, OpState, OpOutcome, LotStatus
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
        
class TestFullProcess(StationProcess):

    def __init__(self, process_model) -> None:
        super().__init__(process_model)
        
        self.STATES = [State(name='init_state'),
            State(name='prep_state', on_enter='initialise_process_data'), 
            State(name='pickup_batch', on_enter=['request_pickup_batch']),
            State(name='run_op', on_enter=['request_to_run_op']),
            State(name='run_analysis_proc', on_enter=['request_analysis_proc']),
            State(name='final_state')]
        
        self.TRANSITIONS = [
            {'source':'init_state','dest':'prep_state'},
            {'source':'prep_state','dest':'pickup_batch'},
            {'source':'pickup_batch','dest':'run_op', 'conditions':'are_req_robot_ops_completed'},
            {'source':'run_op','dest':'run_analysis_proc', 'conditions':'are_req_station_ops_completed'},
            {'source':'run_analysis_proc','dest':'final_state', 'conditions':'are_req_station_procs_completed'}
        ]


    def initialise_process_data(self):
        self.data['batch_index'] = 0

    def request_pickup_batch(self):
        robot_op = RobotOpDescriptor.from_args()
        self.request_robot_ops([robot_op])

    def request_to_run_op(self):
        station_op = self.generate_operation("some_op")
        self.request_station_op(station_op)

    def request_analysis_proc(self):
        station_proc = StationProcess.from_args(self.lot, {})
        self.request_station_process(station_proc)

class HandlerTest(unittest.TestCase):
    def setUp(self) -> None:
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        self.robot_dict = {
            "type": "Robot",
            "location": {"coordinates": [1, 2], "descriptor": "station"},
            "id": 187,
            "batch_capacity":2,
            "handler": "SimRobotOpHandler"
        }

        self.station_dict = {
            'type': 'Station',
            'id': 23,
            "location": {"coordinates": [1, 2], "descriptor": "Station"},
            'handler': 'SimStationOpHandler',
            'total_lot_capacity': 2,
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
        robot.add_op(robot_op)
        self.assertIsNone(robot.assigned_op)
        self.assertEqual(robot.assigned_op_state, OpState.INVALID)

        robot_handler.tick()
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
        self.assertEqual(robot_op.outcome, OpOutcome.SUCCEEDED)
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
        self.assertIsNone(robot.assigned_op)
        
        robot_handler.tick()
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
        self.assertEqual(robot_op.outcome, OpOutcome.SUCCEEDED)

    def test_robot_op_handler_with_skip(self):
        robot = Robot.from_dict(self.robot_dict)
        self.assertEqual(robot.state, RobotState.INACTIVE)
        robot_handler = RobotHandler(robot, use_sim=False)
        robot_handler.initialise()
        self.assertEqual(robot.state, RobotState.ACTIVE)

        robot_op = RobotOpDescriptor.from_args()
        self.assertEqual(robot.assigned_op_state, OpState.INVALID)
        robot.add_op(robot_op)
        self.assertIsNone(robot.assigned_op)

        robot_handler.tick()
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
        self.assertEqual(robot_op.outcome, OpOutcome.SKIPPED)

    def test_robot_op_handler_with_wait_op(self):
        robot = Robot.from_dict(self.robot_dict)
        self.assertEqual(robot.state, RobotState.INACTIVE)
        robot_handler = RobotHandler(robot, use_sim=False)
        robot_handler.initialise()
        self.assertEqual(robot.state, RobotState.ACTIVE)

        robot_op = RobotWaitOpDescriptor.from_args("Robot", 3)
        robot.add_op(robot_op)
        self.assertIsNone(robot.assigned_op)
        self.assertEqual(robot.assigned_op_state, OpState.INVALID)

        robot_handler.tick()
        self.assertIsNotNone(robot.assigned_op)
        self.assertEqual(robot.assigned_op_state, OpState.ASSIGNED)
        self.assertIsNone(robot_op.start_timestamp)
        self.assertIsNone(robot_op.end_timestamp)

        robot_handler.tick()
        self.assertEqual(robot.assigned_op_state, OpState.EXECUTING)
        self.assertEqual(robot_op.outcome, OpOutcome.SUCCEEDED)
        self.assertIsNotNone(robot_op.start_timestamp)
        self.assertIsNotNone(robot.assigned_op)

        robot_handler.tick()
        self.assertEqual(robot.assigned_op_state, OpState.EXECUTING)
        self.assertIsNotNone(robot.assigned_op)
        self.assertEqual(robot_op.outcome, OpOutcome.SUCCEEDED)

        robot_handler.tick()
        self.assertEqual(robot.assigned_op_state, OpState.EXECUTING)
        self.assertIsNotNone(robot.assigned_op)
        self.assertEqual(robot_op.outcome, OpOutcome.SUCCEEDED)

        
        sleep(3) # the wait op should be complete
        robot_handler.tick()
        self.assertIsNone(robot.assigned_op)
        self.assertEqual(robot.assigned_op_state, OpState.INVALID)
        self.assertEqual(len(robot.ops_history), 1)

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
        self.assertEqual(station_op.outcome, OpOutcome.SUCCEEDED)
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
        self.assertEqual(station_op.outcome, OpOutcome.SUCCEEDED)
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
        self.assertEqual(station_op.outcome, OpOutcome.SKIPPED)
        self.assertIsNotNone(station_op.end_timestamp)

    def test_station_process_handler_with_added_procs(self):
        # construct lots
        batch_1 = Batch.from_args(3, Location.from_args(coordinates=(1,2), descriptor="some_frame"))
        batch_2 = Batch.from_args(3, Location.from_args(coordinates=(1,2), descriptor="some_frame"))
        lot_1 = Lot.from_args([batch_1])
        lot_2 = Lot.from_args([batch_2])
        
        # construct process
        proc_1 = TestProcess.from_args(lot_1)
        proc_2 = TestProcess.from_args(lot_2)

        # construct station and its process handler
        station = Station.from_dict(self.station_dict)
        proc_handler = StationProcessHandler(station)

        self.assertFalse(station.running_procs)
        self.assertFalse(station.queued_procs)

        # add procs to station
        station.add_process(proc_1)
        station.add_process(proc_2)
        self.assertEqual(len(station.queued_procs), 2)
        self.assertFalse(station.running_procs)
        
        # test processes assignment
        proc_handler.handle()
        self.assertEqual(len(station.running_procs), 2)
        self.assertEqual(station.running_procs[0].status, ProcessStatus.INACTIVE)
        self.assertEqual(station.running_procs[1].status, ProcessStatus.INACTIVE)
        self.assertEqual(station.running_procs[0].object_id, proc_1.object_id)
        self.assertEqual(station.running_procs[1].object_id, proc_2.object_id)
        self.assertEqual(len(station.queued_procs), 0)

        # test processes state advancement
        proc_handler.handle()
        self.assertEqual(station.running_procs[0].m_state, "some_state")
        self.assertEqual(station.running_procs[1].m_state, "some_state")
        self.assertEqual(station.running_procs[0].status, ProcessStatus.RUNNING)
        self.assertEqual(station.running_procs[1].status, ProcessStatus.RUNNING)

        # test processes state advancement
        proc_handler.handle()
        self.assertEqual(station.running_procs[0].m_state, "final_state")
        self.assertEqual(station.running_procs[1].m_state, "final_state")
        self.assertEqual(station.running_procs[0].status, ProcessStatus.FINISHED)
        self.assertEqual(station.running_procs[1].status, ProcessStatus.FINISHED)

         # test processes completion
        proc_handler.handle()
        self.assertFalse(station.running_procs)
        self.assertEqual(proc_1.status, ProcessStatus.FINISHED)
        self.assertEqual(proc_2.status, ProcessStatus.FINISHED)
        self.assertEqual(len(station.procs_history), 2)

    def test_station_process_handler_with_added_lots(self):
        # construct lots and their recipes
        batch_1 = Batch.from_args(3, Location.from_args(coordinates=(1,2), descriptor="some_frame"))
        batch_2 = Batch.from_args(3, Location.from_args(coordinates=(1,2), descriptor="some_frame"))
        lot_1 = Lot.from_args([batch_1])
        lot_1.status = LotStatus.IN_WORKFLOW
        lot_2 = Lot.from_args([batch_2])
        lot_2.status = LotStatus.IN_WORKFLOW
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

        self.assertFalse(station.running_procs)
        self.assertFalse(station.queued_procs)

        # add procs to station
        station.add_lot(lot_1)
        station.add_lot(lot_2)
       
        self.assertFalse(station.queued_procs)

        # test processes assignment
        proc_handler.handle()
        self.assertEqual(len(station.running_procs), 2)
        self.assertEqual(station.running_procs[0].status, ProcessStatus.INACTIVE)
        self.assertEqual(station.running_procs[1].status, ProcessStatus.INACTIVE)

        # test processes state advancement
        proc_handler.handle()
        self.assertEqual(station.running_procs[0].m_state, "final_state")
        self.assertEqual(station.running_procs[1].m_state, "final_state")
        self.assertEqual(station.running_procs[0].status, ProcessStatus.FINISHED)
        self.assertEqual(station.running_procs[1].status, ProcessStatus.FINISHED)

        # test processes completion
        proc_handler.handle()
        self.assertFalse(station.running_procs)
        self.assertEqual(len(station.procs_history), 2)

    def test_station_process_handler_mixed(self):
        # construct lots and their recipes
        batch_1 = Batch.from_args(3, Location.from_args(coordinates=(1,2), descriptor="some_frame"))
        batch_2 = Batch.from_args(3, Location.from_args(coordinates=(1,2), descriptor="some_frame"))
        lot_1 = Lot.from_args([batch_1])
        lot_1.status = LotStatus.IN_WORKFLOW
        lot_2 = Lot.from_args([batch_2])
        lot_2.status = LotStatus.IN_WORKFLOW
        recipe_1_doc = self.recipe_doc
        recipe_1 = Recipe.from_dict(recipe_1_doc)
        lot_1.attach_recipe(recipe_1)

        recipe_2_doc = self.recipe_doc
        recipe_2_doc["general"]["id"] = 199
        recipe_2 = Recipe.from_dict(recipe_2_doc)
        lot_2.attach_recipe(recipe_2)

        # construct process
        batch_3 = Batch.from_args(3, Location.from_args(coordinates=(1,2), descriptor="some_frame"))
        lot_3 = Lot.from_args([batch_3])
        lot_3.status = LotStatus.IN_WORKFLOW
        proc = TestProcess.from_args(lot_3)

        # construct station and its process handler
        station = Station.from_dict(self.station_dict)
        proc_handler = StationProcessHandler(station)

        self.assertFalse(station.running_procs)
        self.assertFalse(station.queued_procs)

        # add lots and procs to station
        station.add_lot(lot_1)
        station.add_process(proc)
       
        self.assertEqual(len(station.queued_procs), 1)
        
        # test processes assignment
        proc_handler.handle()
        self.assertEqual(len(station.queued_procs), 0)
        self.assertEqual(len(station.running_procs), 2)
        self.assertEqual(station.running_procs[0].object_id, proc.object_id)
        self.assertEqual(station.running_procs[0].status, ProcessStatus.INACTIVE)
        self.assertEqual(station.running_procs[1].status, ProcessStatus.INACTIVE)

        # add a lot
        station.add_lot(lot_2)

        # test processes state advancement
        proc_handler.handle()
        self.assertEqual(len(station.queued_procs), 1)
        self.assertEqual(station.running_procs[0].m_state, "some_state")
        self.assertEqual(station.running_procs[1].m_state, "final_state")
        self.assertEqual(station.running_procs[0].status, ProcessStatus.RUNNING)
        self.assertEqual(station.running_procs[1].status, ProcessStatus.FINISHED)

        # test processes state advancement
        proc_handler.handle()
        self.assertEqual(station.running_procs[0].m_state, "final_state")
        self.assertEqual(station.running_procs[0].status, ProcessStatus.FINISHED)
        self.assertEqual(station.running_procs[1].m_state, "init_state")
        self.assertEqual(station.running_procs[1].status, ProcessStatus.INACTIVE)
        self.assertFalse(station.queued_procs)
        self.assertEqual(len(station.running_procs), 2)
        self.assertEqual(len(station.procs_history), 1)

        # test processes state advancement
        proc_handler.handle()
        self.assertEqual(proc.status, ProcessStatus.FINISHED)
        self.assertEqual(len(station.running_procs), 1)
        self.assertEqual(station.running_procs[0].m_state, "final_state")
        self.assertEqual(station.running_procs[0].status, ProcessStatus.FINISHED)
        self.assertEqual(len(station.procs_history), 2)

        # test processes completion
        proc_handler.handle()
        self.assertFalse(station.queued_procs)
        self.assertFalse(station.running_procs)
        self.assertEqual(len(station.procs_history), 3)

        # test again processes completion
        proc_handler.handle()
        self.assertFalse(station.queued_procs)
        self.assertFalse(station.running_procs)
        self.assertEqual(len(station.procs_history), 3)

    def test_station_process_handler_with_requests(self):
        # construct lots
        batch_1 = Batch.from_args(3, Location.from_args(coordinates=(1,2), descriptor="some_frame"))
        batch_2 = Batch.from_args(3, Location.from_args(coordinates=(1,2), descriptor="some_frame"))
        lot_1 = Lot.from_args([batch_1])
        lot_2 = Lot.from_args([batch_2])
        
        # construct process
        operations = [
                {
                    "name": "some_op",
                    "op": "StationOpDescriptor",
                    "parameters": None
                }
            ]
        proc_1 = TestFullProcess.from_args(lot_1, operations)
        proc_2 = TestFullProcess.from_args(lot_2, operations)

        # construct station and its process handler
        station = Station.from_dict(self.station_dict)
        self.assertFalse(station.requested_robot_ops)
        self.assertFalse(station.requested_ext_procs)
        self.assertFalse(station._queued_ops)
        proc_handler = StationProcessHandler(station)

        self.assertFalse(station.running_procs)
        self.assertFalse(station.queued_procs)

        # add procs to station
        station.add_process(proc_1)
        station.add_process(proc_2)
        self.assertEqual(len(station.queued_procs), 2)
        
        # test processes assignment
        proc_handler.handle()
        self.assertEqual(station.running_procs[0].object_id, proc_1.object_id)
        self.assertEqual(station.running_procs[1].object_id, proc_2.object_id)
        self.assertEqual(len(station.queued_procs), 0)

        # test processes state advancement
        proc_handler.handle()
        self.assertEqual(station.running_procs[0].m_state, "prep_state")
        self.assertEqual(station.running_procs[1].m_state, "prep_state")
        self.assertEqual(station.running_procs[0].status, ProcessStatus.RUNNING)
        self.assertEqual(station.running_procs[1].status, ProcessStatus.RUNNING)

        # test processes state advancement
        proc_handler.handle()
        self.assertEqual(station.running_procs[0].m_state, "pickup_batch")
        self.assertEqual(station.running_procs[1].m_state, "pickup_batch")
        self.assertEqual(station.running_procs[0].status, ProcessStatus.WAITING_ON_ROBOT_OPS)
        self.assertEqual(station.running_procs[1].status, ProcessStatus.WAITING_ON_ROBOT_OPS)
        self.assertEqual(len(station.requested_robot_ops), 2)
        robot_op = station.requested_robot_ops.pop(left=True)
        robot_op.complete_op(None, OpOutcome.SUCCEEDED)

        # test processes state advancement
        proc_handler.handle()
        self.assertEqual(station.running_procs[0].m_state, "run_op")
        self.assertEqual(station.running_procs[1].m_state, "pickup_batch")
        self.assertEqual(station.running_procs[0].status, ProcessStatus.WAITING_ON_STATION_OPS)
        self.assertEqual(station.running_procs[1].status, ProcessStatus.WAITING_ON_ROBOT_OPS)
        self.assertEqual(len(station.requested_robot_ops), 1)
        self.assertEqual(len(station._queued_ops), 1)
        station_op = station._queued_ops.pop()
        station_op.complete_op(OpOutcome.SUCCEEDED, None)

        # test processes state advancement
        proc_handler.handle()
        self.assertEqual(station.running_procs[0].m_state, "run_analysis_proc")
        self.assertEqual(station.running_procs[1].m_state, "pickup_batch")
        self.assertEqual(station.running_procs[0].status, ProcessStatus.WAITING_ON_STATION_PROCS)
        self.assertEqual(len(station.requested_ext_procs), 1)
        station_proc = station.requested_ext_procs.pop()
        station_proc._model_proxy.status = ProcessStatus.FINISHED

        # test processes state advancement
        proc_handler.handle()
        self.assertEqual(station.running_procs[0].m_state, "final_state")
        self.assertEqual(station.running_procs[0].status, ProcessStatus.FINISHED)
        self.assertEqual(station.running_procs[1].m_state, "pickup_batch")

        # test processes state advancement
        proc_handler.handle()
        self.assertEqual(len(station.running_procs), 1)
        self.assertEqual(station.running_procs[0].m_state, "pickup_batch")



if __name__ == "__main__":
    unittest.main()
        