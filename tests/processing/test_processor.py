import unittest
from mongoengine import connect
from transitions import State

from archemist.core.state.station import Station
from archemist.core.state.station_process import StationProcess
from archemist.core.state.robot_op import RobotOpDescriptor
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.state.recipe import Recipe
from archemist.core.state.state import InputState, WorkflowState, OutputState
from archemist.core.processing.processor import InputProcessor, OutputProcessor, WorkflowProcessor
from archemist.core.processing.handler import StationProcessHandler
from archemist.core.util.enums import LotStatus, OpOutcome

class TestProcess1(StationProcess):

    def __init__(self, process_model) -> None:
        super().__init__(process_model)
        
        self.STATES = [State(name='init_state'),
            State(name='pickup_batch', on_enter=['request_pickup_batch']),
            State(name='final_state')]
        
        self.TRANSITIONS = [
            {'source':'init_state','dest':'pickup_batch'},
            {'source':'pickup_batch','dest':'final_state', 'conditions':'are_req_robot_ops_completed'},
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

class TestProcess2(StationProcess):

    def __init__(self, process_model) -> None:
        super().__init__(process_model)
        
        self.STATES = [State(name='init_state'),
            State(name='pickup_batch', on_enter=['request_pickup_batch']),
            State(name='run_analysis_proc', on_enter=['request_analysis_proc']),
            State(name='final_state')]
        
        self.TRANSITIONS = [
            {'source':'init_state','dest':'pickup_batch'},
            {'source':'pickup_batch','dest':'run_analysis_proc', 'conditions':'are_req_robot_ops_completed'},
            {'source':'run_analysis_proc','dest':'final_state', 'conditions':'are_req_station_procs_completed'}
        ]

    def request_pickup_batch(self):
        robot_op = RobotOpDescriptor.from_args()
        self.request_robot_ops([robot_op])

    def request_analysis_proc(self):
        station_proc = StationProcess.from_args(self.lot, {})
        self.request_station_process(station_proc)

class ProcessorTest(unittest.TestCase):

    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')


        self.recipe_doc_1 = {
            "general": {"name": "test_archemist_recipe", "id": 198},
            "steps": [
                {
                    "state_name": "step_1",
                    "station": {
                        "type": "Station",
                        "id": 23,
                        "process": {
                            "type": "TestProcess1",
                            "operations": None,
                            "args": None,
                        },
                    },
                    "transitions": {
                        "on_success": "step_2",
                        "on_fail": "failed_state",
                    },
                },
                {
                    "state_name": "step_2",
                    "station": {
                        "type": "Station",
                        "id": 32,
                        "process": {
                            "type": "TestProcess2",
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

        self.recipe_doc_2 = {
            "general": {"name": "test_archemist_recipe", "id": 199},
            "steps": [
                {
                    "state_name": "step_1",
                    "station": {
                        "type": "Station",
                        "id": 23,
                        "process": {
                            "type": "TestProcess1",
                            "operations": None,
                            "args": None,
                        },
                    },
                    "transitions": {
                        "on_success": "step_2",
                        "on_fail": "failed_state",
                    },
                },
                {
                    "state_name": "step_2",
                    "station": {
                        "type": "Station",
                        "id": 32,
                        "process": {
                            "type": "TestProcess2",
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

        self.operations = [{
            "name": "some_op",
            "op": "StationOpDescriptor",
            "parameters": {"stirring_speed": 200, "duration": 10}
        }]

        self.station_1_dict = {
            'type': 'Station',
            'id': 23,
            "location": {"coordinates": [1, 7], "descriptor": "Station1"},
            'handler': 'SimStationOpHandler',
            'total_lot_capacity': 2
        }

        self.station_2_dict = {
            'type': 'Station',
            'id': 32,
            "location": {"coordinates": [2, 7], "descriptor": "Station2"},
            'handler': 'SimStationOpHandler',
            'total_lot_capacity': 2
        }


    def tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_input_processor(self):

        batch = Batch.from_args(3)
        lot = Lot.from_args([batch])
        
        station = Station.from_dict(self.station_1_dict)
        station.add_lot(lot)
        self.assertEqual(station.free_lot_capacity, 1)

        input_dict = {
            "location": {"coordinates": [1, 7], "descriptor": "InputSite"},
            "samples_per_batch": 3,
            "batches_per_lot": 1,
            "total_lot_capacity": 2,
            "lot_input_process": {
                "type": "StationProcess",
                "args": None
            }
        }
        input_state = InputState.from_dict(input_dict)
        input_processor = InputProcessor(input_state)

        # test requested_robot_ops
        self.assertFalse(input_processor.requested_robot_ops)
        input_processor.requested_robot_ops.append(RobotOpDescriptor.from_args())
        self.assertEqual(len(input_processor.requested_robot_ops), 1)

        # test adding clean batches
        self.assertFalse(input_state.batches_queue)
        input_processor.add_clean_batch()
        self.assertEqual(len(input_state.batches_queue), 1)
        input_processor.add_clean_batch()
        self.assertEqual(len(input_state.batches_queue), 2)
        input_processor.add_clean_batch() # no batch created since at capacity
        self.assertEqual(len(input_state.batches_queue), 2)

        # test adding new recipes
        self.assertFalse(input_state.recipes_queue)
        input_processor.add_recipe(self.recipe_doc_1)
        self.assertEqual(len(input_state.recipes_queue), 1)
        input_processor.add_recipe(self.recipe_doc_2)
        self.assertEqual(len(input_state.recipes_queue), 2)
        input_processor.add_recipe(self.recipe_doc_2) # recipe not added since recipe already added
        self.assertEqual(len(input_state.recipes_queue), 2)

        # test process lots
        self.assertFalse(input_state.procs_history)
        self.assertFalse(input_processor.retrieve_ready_for_collection_lots())

        self.assertEqual(input_state.get_lots_num(), 0)
        input_processor.process_lots()
        self.assertEqual(input_state.get_lots_num(), 2)
        self.assertEqual(input_state.lot_slots["0"].status, LotStatus.CREATED)
        self.assertFalse(input_state.lot_slots["0"].is_recipe_attached())
        self.assertEqual(input_state.lot_slots["1"].status, LotStatus.CREATED)
        self.assertFalse(input_state.lot_slots["1"].is_recipe_attached())

        input_processor.process_lots()
        self.assertFalse(input_state.recipes_queue)
        self.assertEqual(input_state.lot_slots["0"].status, LotStatus.STANDBY)
        self.assertTrue(input_state.lot_slots["0"].is_recipe_attached())
        self.assertEqual(input_state.lot_slots["1"].status, LotStatus.STANDBY)
        self.assertTrue(input_state.lot_slots["1"].is_recipe_attached())

        input_processor.process_lots()
        self.assertEqual(input_state.lot_slots["0"].status, LotStatus.ONBOARDING)
        input_proc = input_state.proc_slots["0"]
        self.assertIsNotNone(input_proc)
        self.assertEqual(input_proc.m_state, "init_state")
        self.assertEqual(input_state.lot_slots["1"].status, LotStatus.STANDBY)
        self.assertIsNone(input_state.proc_slots["1"])

        input_processor.process_lots()
        self.assertEqual(input_state.lot_slots["0"].status, LotStatus.ONBOARDING)
        self.assertEqual(input_proc.m_state, "final_state")
        self.assertEqual(input_state.lot_slots["1"].status, LotStatus.STANDBY)
        self.assertIsNone(input_state.proc_slots["1"])

        input_processor.process_lots()
        self.assertEqual(input_state.lot_slots["0"].status, LotStatus.READY_FOR_COLLECTION)
        self.assertIsNone(input_state.proc_slots["0"])
        self.assertEqual(len(input_state.procs_history), 1)
        self.assertEqual(input_state.lot_slots["1"].status, LotStatus.STANDBY)
        self.assertIsNone(input_state.proc_slots["1"])

        # test retrieving ready for collection lots
        rdy_for_collection_lots = input_processor.retrieve_ready_for_collection_lots()
        self.assertEqual(len(rdy_for_collection_lots), 1)
        self.assertIsNone(input_state.lot_slots["0"])
        self.assertIsNotNone(input_state.lot_slots["1"])
        self.assertEqual(input_state.get_lots_num(), 1)

    def test_output_processor(self):
        output_dict = {
            "location": {"coordinates": [1, 7], "descriptor": "OutputSite"},
            "total_lot_capacity": 2,
            "lot_output_process": None,
            "lots_need_manual_removal": True
        }

        output_state = OutputState.from_dict(output_dict)
        output_processor = OutputProcessor(output_state)

        self.assertEqual(output_state.get_lots_num(), 0)
        self.assertTrue(output_processor.has_free_lot_capacity())

        # test requested_robot_ops
        self.assertFalse(output_processor.requested_robot_ops)
        output_processor.requested_robot_ops.append(RobotOpDescriptor.from_args())
        self.assertEqual(len(output_processor.requested_robot_ops), 1)

        # create lots
        batch_1 = Batch.from_args(3)
        lot_1 = Lot.from_args([batch_1])
        lot_1.status = LotStatus.IN_WORKFLOW

        batch_2 = Batch.from_args(3)
        lot_2 = Lot.from_args([batch_2])
        lot_2.status = LotStatus.IN_WORKFLOW

        batch_3 = Batch.from_args(3)
        lot_3 = Lot.from_args([batch_3])

        # test adding lots
        output_processor.add_lot(lot_1)
        self.assertEqual(output_state.get_lots_num(), 1)
        self.assertTrue(output_processor.has_free_lot_capacity())

        output_processor.add_lot(lot_2)
        self.assertEqual(output_state.get_lots_num(), 2)
        self.assertFalse(output_processor.has_free_lot_capacity())

        output_processor.add_lot(lot_3) # lot not added since no slot available
        self.assertEqual(output_state.get_lots_num(), 2)

        # test processing lots
        self.assertFalse(output_state.procs_history)

        output_processor.process_lots()
        self.assertEqual(output_state.lot_slots["0"].status, LotStatus.OFFBOARDING)
        self.assertIsNotNone(output_state.proc_slots["0"])
        self.assertEqual(output_state.proc_slots["0"].m_state, "init_state")
        self.assertEqual(output_state.lot_slots["1"].status, LotStatus.OFFBOARDING)
        self.assertIsNotNone(output_state.proc_slots["1"])
        self.assertEqual(output_state.proc_slots["1"].m_state, "init_state")

        output_processor.process_lots()
        self.assertEqual(output_state.lot_slots["0"].status, LotStatus.OFFBOARDING)
        self.assertEqual(output_state.proc_slots["0"].m_state, "final_state")
        self.assertEqual(output_state.lot_slots["1"].status, LotStatus.OFFBOARDING)
        self.assertEqual(output_state.proc_slots["1"].m_state, "final_state")

        output_processor.process_lots()
        self.assertEqual(output_state.lot_slots["0"].status, LotStatus.NEED_REMOVAL)
        self.assertIsNone(output_state.proc_slots["0"])
        self.assertEqual(output_state.lot_slots["1"].status, LotStatus.NEED_REMOVAL)
        self.assertIsNone(output_state.proc_slots["1"])

        output_processor.remove_lot("0")
        self.assertIsNone(output_state.lot_slots["0"])
        self.assertEqual(output_state.lot_slots["1"].status, LotStatus.NEED_REMOVAL)

        output_processor.remove_all_lots()
        self.assertIsNone(output_state.lot_slots["0"])
        self.assertIsNone(output_state.lot_slots["1"])

    def test_workflow_processor(self):

        # create lots and attach recipes
        recipe_1 = Recipe.from_dict(self.recipe_doc_1)
        batch_1 = Batch.from_args(3)
        lot_1 = Lot.from_args([batch_1])
        lot_1.attach_recipe(recipe_1)
        lot_1.status = LotStatus.READY_FOR_COLLECTION

        recipe_2 = Recipe.from_dict(self.recipe_doc_2)
        batch_2 = Batch.from_args(3)
        lot_2 = Lot.from_args([batch_2])
        lot_2.attach_recipe(recipe_2)
        lot_2.status = LotStatus.READY_FOR_COLLECTION

        # create stations and their proc handlers
        station_1 = Station.from_dict(self.station_1_dict)
        station_1_proc_handler = StationProcessHandler(station_1)
        station_2 = Station.from_dict(self.station_2_dict)
        station_2_proc_handler = StationProcessHandler(station_2)

        # create workflow processor
        workflow_state = WorkflowState.from_args("test_workflow")
        workflow_processor = WorkflowProcessor(workflow_state)

        # test lots buffer
        self.assertFalse(workflow_state.lots_buffer)
        
        # test adding lots
        workflow_processor.add_ready_for_collection_lots([lot_1, lot_2])
        self.assertEqual(len(workflow_state.lots_buffer), 2)
        self.assertEqual(workflow_state.lots_buffer[0], lot_1)
        self.assertEqual(workflow_state.lots_buffer[1], lot_2)

        # test workflow processor main loop
        # test correct lot assignment
        workflow_processor.process_workflow()
        self.assertEqual(station_1.free_lot_capacity, 0)
        self.assertEqual(station_2.free_lot_capacity, 2)
        self.assertFalse(workflow_state.lots_buffer)

        # create processes and add them to the station 1 (this is done to bypass using ProcessFactory)
        proc_1 = TestProcess1.from_args(lot_1, self.operations)
        proc_2 = TestProcess1.from_args(lot_2, self.operations)
        station_1.add_process(proc_1)
        station_1.add_process(proc_2)
        lot_1.status = LotStatus.IN_PROCESS
        lot_2.status = LotStatus.IN_PROCESS

        # tick station_1_proc_handler to assigns proc slots
        station_1_proc_handler.handle()
        self.assertEqual(station_1.running_procs[0].object_id, proc_1.object_id)
        self.assertEqual(station_1.running_procs[0].m_state, "init_state")
        self.assertEqual(station_1.running_procs[1].object_id, proc_2.object_id)
        self.assertEqual(station_1.running_procs[1].m_state, "init_state")

        #  tick station_1_proc_handler to advance proc states
        station_1_proc_handler.handle()
        self.assertEqual(station_1.running_procs[0].m_state, "pickup_batch")
        self.assertEqual(station_1.running_procs[1].m_state, "pickup_batch")
        self.assertEqual(len(station_1.requested_robot_ops), 2)

        # tick workflow processor to pick requested robot ops from station 1
        workflow_processor.process_workflow()
        self.assertFalse(station_1.requested_robot_ops)
        req_robot_ops = workflow_processor.robot_ops_queue
        self.assertEqual(len(req_robot_ops), 2)

        # tick station_1_proc_handler to sanity check it doesn't advance state
        station_1_proc_handler.handle()
        self.assertEqual(station_1.running_procs[0].m_state, "pickup_batch")
        self.assertEqual(station_1.running_procs[1].m_state, "pickup_batch")
        self.assertFalse(station_1.requested_robot_ops)

        # complete req robot ops to be able to advance procs
        req_robot_op_1 = req_robot_ops.pop(left=True)
        req_robot_op_1.complete_op(None, OpOutcome.SUCCEEDED)
        req_robot_op_2 = req_robot_ops.pop(left=True)
        req_robot_op_2.complete_op(None, OpOutcome.SUCCEEDED)

        # tick station_1_proc_handler to advance state
        station_1_proc_handler.handle()
        self.assertEqual(station_1.running_procs[0].m_state, "final_state")
        self.assertEqual(station_1.running_procs[1].m_state, "final_state")

        # tick station_1_proc_handler to finish
        station_1_proc_handler.handle()
        self.assertFalse(station_1.queued_procs)
        self.assertFalse(station_1.running_procs)
        self.assertTrue(station_1.has_ready_for_collection_lots())

        # tick workflow processor to pick processed lots from station 1
        workflow_processor.process_workflow()
        self.assertEqual(station_1.free_lot_capacity, 2)
        self.assertEqual(len(workflow_state.lots_buffer), 2)

        # tick workflow processor to add lots to station 2
        workflow_processor.process_workflow()
        self.assertEqual(station_1.free_lot_capacity, 2)
        self.assertEqual(station_2.free_lot_capacity, 0)
        self.assertFalse(workflow_state.lots_buffer)

        # create processes and add them to the station 1 (this is done to bypass using ProcessFactory)
        proc_1 = TestProcess2.from_args(lot_1)
        proc_2 = TestProcess2.from_args(lot_2)
        station_2.add_process(proc_1)
        station_2.add_process(proc_2)
        lot_1.status = LotStatus.IN_PROCESS
        lot_2.status = LotStatus.IN_PROCESS

        # tick station_2_proc_handler to assigns proc slots
        station_2_proc_handler.handle()
        self.assertEqual(station_2.running_procs[0].object_id, proc_1.object_id)
        self.assertEqual(station_2.running_procs[0].m_state, "init_state")
        self.assertEqual(station_2.running_procs[1].object_id, proc_2.object_id)
        self.assertEqual(station_2.running_procs[1].m_state, "init_state")

        # tick station_2_proc_handler to advance proc states
        station_2_proc_handler.handle()
        self.assertEqual(station_2.running_procs[0].m_state, "pickup_batch")
        self.assertEqual(station_2.running_procs[1].m_state, "pickup_batch")
        self.assertEqual(len(station_2.requested_robot_ops), 2)

        # tick workflow processor to pick robot ops
        workflow_processor.process_workflow()
        self.assertFalse(station_2.requested_robot_ops)
        req_robot_ops = workflow_processor.robot_ops_queue
        self.assertEqual(len(req_robot_ops), 2)

        # complete req robot ops to be able to advance procs
        req_robot_op_1 = req_robot_ops.pop(left=True)
        req_robot_op_1.complete_op(None, OpOutcome.SUCCEEDED)
        req_robot_op_2 = req_robot_ops.pop(left=True)
        req_robot_op_2.complete_op(None, OpOutcome.SUCCEEDED)

        # tick station_2_proc_handler to advance proc states
        station_2_proc_handler.handle()
        self.assertEqual(station_2.running_procs[0].m_state, "run_analysis_proc")
        self.assertEqual(station_2.running_procs[1].m_state, "run_analysis_proc")
        self.assertEqual(len(station_2.requested_ext_procs), 2)

        # tick workflow processor to pick requested external processes
        workflow_processor.process_workflow()
        self.assertFalse(station_2.requested_ext_procs)
        self.assertEqual(len(station_1.queued_procs), 2)

        # tick station_1_proc_handler to advance proc states
        station_1_proc_handler.handle()
        self.assertIsNotNone(station_1.running_procs[0])
        self.assertEqual(station_1.running_procs[0].m_state, "init_state")
        self.assertIsNotNone(station_1.running_procs[1])
        self.assertEqual(station_1.running_procs[1].m_state, "init_state")

        # tick station_1_proc_handler to advance proc states
        station_1_proc_handler.handle()
        self.assertEqual(station_1.running_procs[0].m_state, "final_state")
        self.assertEqual(station_1.running_procs[1].m_state, "final_state")

        # tick station_1_proc_handler to finish procs
        station_1_proc_handler.handle()
        self.assertFalse(station_1.running_procs)

        # tick station_2_proc_handler to advance proc states
        station_2_proc_handler.handle()
        self.assertEqual(station_2.running_procs[0].m_state, "final_state")
        self.assertEqual(station_2.running_procs[1].m_state, "final_state")

        # tick station_2_proc_handler to finish processes
        station_2_proc_handler.handle()
        self.assertFalse(station_2.running_procs)
        self.assertTrue(station_2.has_ready_for_collection_lots())

        # tick workflow processor to pick processed lots from station 2
        workflow_processor.process_workflow()
        self.assertEqual(station_2.free_lot_capacity, 2)
        self.assertEqual(len(workflow_state.lots_buffer), 2)

        # tick workflow processor to ensure that lot recipes are complete
        workflow_processor.process_workflow()
        self.assertEqual(len(workflow_state.lots_buffer), 2)

        # test retrieve complete lots
        completed_lot = workflow_processor.retrieve_completed_lot()
        self.assertIsNotNone(completed_lot)
        self.assertEqual(len(workflow_state.lots_buffer), 1)

        completed_lot = workflow_processor.retrieve_completed_lot()
        self.assertIsNotNone(completed_lot)
        self.assertEqual(len(workflow_state.lots_buffer), 0)











if __name__ == "__main__":
    unittest.main()


