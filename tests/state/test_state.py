import unittest
from datetime import date
from archemist.core.util.location import Location
from archemist.core.state.robot_op import RobotOp
from archemist.core.state.station_op import StationOp
from archemist.core.state.station_process import StationProcess
from archemist.core.state.recipe import Recipe
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot, LotStatus
from archemist.core.state.state import InputState, WorkflowState, OutputState
from mongoengine import connect


class StateTest(unittest.TestCase):

    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

    def tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_input_state(self):
        input_dict = {
            "location":  {'coordinates': [1, 7], 'descriptor': 'InputSite'},
            "samples_per_batch": 3,
            "batches_per_lot": 1,
            "total_lot_capacity": 2,
            "lot_input_process": {
                "type": "StationProcess",
                "args": None
            }
        }
        input_state = InputState.from_dict(input_dict)
        self.assertIsNotNone(input_state.object_id)
        self.assertEqual(input_state.location, Location.from_dict({'coordinates': [1, 7], 'descriptor': 'InputSite'}))
        self.assertEqual(input_state.samples_per_batch, 3)
        self.assertEqual(input_state.batches_per_lot, 1)
        self.assertEqual(input_state.total_lot_capacity, 2)
        self.assertEqual(input_state.lot_input_process["type"], input_dict["lot_input_process"]["type"])
        self.assertEqual(input_state.lot_input_process["args"], input_dict["lot_input_process"]["args"])

        # test empty queues
        self.assertFalse(input_state.batches_queue)
        self.assertFalse(input_state.recipes_queue)
        self.assertFalse(input_state.requested_robot_ops)
        self.assertFalse(input_state.procs_history)

        # test empty lot_slots
        self.assertEqual(len(input_state.lot_slots), 2)
        self.assertIsNone(input_state.lot_slots["0"])
        self.assertIsNone(input_state.lot_slots["1"])
        self.assertEqual(input_state.get_lots_num(), 0)

        # test empty proc slots
        self.assertEqual(len(input_state.proc_slots), 2)
        self.assertIsNone(input_state.proc_slots["0"])
        self.assertIsNone(input_state.proc_slots["1"])

        # test batches_queue
        input_state.batches_queue.append(Batch.from_args(input_state.samples_per_batch, input_state.location))
        self.assertEqual(len(input_state.batches_queue), 1)
        input_state.batches_queue.append(Batch.from_args(input_state.samples_per_batch, input_state.location))
        self.assertEqual(len(input_state.batches_queue), 2)

        # test lot_slots
        batch_1 = input_state.batches_queue.pop(left=True)
        self.assertIsNotNone(batch_1)
        self.assertEqual(len(input_state.batches_queue), 1)
        lot_1 = Lot.from_args([batch_1])
        input_state.lot_slots["0"] = lot_1
        self.assertEqual(input_state.get_lots_num(), 1)
        self.assertIsNotNone(input_state.lot_slots["0"])

        batch_2 = input_state.batches_queue.pop(left=True)
        self.assertIsNotNone(batch_2)
        self.assertEqual(len(input_state.batches_queue), 0)
        lot_2 = Lot.from_args([batch_2])
        input_state.lot_slots["1"] = lot_2
        self.assertEqual(input_state.get_lots_num(), 2)
        self.assertIsNotNone(input_state.lot_slots["1"])

        lot_2.status = LotStatus.READY_FOR_COLLECTION
        self.assertEqual(input_state.get_lots_num(status=LotStatus.READY_FOR_COLLECTION), 1)

        # set lots to None
        input_state.lot_slots["1"] = None
        self.assertIsNone(input_state.lot_slots["1"])

        # test recipes queue
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

        recipe_1 = Recipe.from_dict(recipe_doc_1)

        input_state.recipes_queue.append(recipe_1)
        self.assertEqual(len(input_state.recipes_queue), 1)
        self.assertIsNotNone(input_state.recipes_queue.pop())
        self.assertEqual(len(input_state.recipes_queue), 0)

        # test requested_robot_ops
        robot_op = RobotOp.from_args()
        input_state.requested_robot_ops.append(robot_op)
        self.assertEqual(len(input_state.requested_robot_ops), 1)
        self.assertIsNotNone(input_state.requested_robot_ops.pop())
        self.assertEqual(len(input_state.requested_robot_ops), 0)

        # test proc_slots
        proc = StationProcess.from_args(lot_1)
        input_state.proc_slots["0"] = proc
        self.assertIsNotNone(input_state.proc_slots["0"])
        input_state.proc_slots["0"] = None
        self.assertIsNone(input_state.proc_slots["0"])

        # test proc history
        input_state.procs_history.append(proc)
        self.assertEqual(len(input_state.procs_history), 1)
        self.assertIsNotNone(input_state.procs_history.pop())
        self.assertEqual(len(input_state.procs_history), 0)

    def test_workflow_state(self):
        workflow_state = WorkflowState.from_args("test_workflow")
        self.assertEqual(workflow_state.workflow_name, "test_workflow")

        # test lots buffer
        self.assertFalse(workflow_state.lots_buffer)

        self.assertIsNotNone(workflow_state.object_id)
        batch = Batch.from_args(3)
        lot = Lot.from_args([batch])
        workflow_state.lots_buffer.append(lot)
        self.assertEqual(len(workflow_state.lots_buffer), 1)
        self.assertIsNotNone(workflow_state.lots_buffer.pop())
        self.assertEqual(len(workflow_state.lots_buffer), 0)

        # test requested_robot_ops
        self.assertFalse(workflow_state.robot_ops_queue)

        robot_op = RobotOp.from_args()
        workflow_state.robot_ops_queue.append(robot_op)
        self.assertEqual(len(workflow_state.robot_ops_queue), 1)
        self.assertIsNotNone(workflow_state.robot_ops_queue.pop())
        self.assertEqual(len(workflow_state.robot_ops_queue), 0)

        # test station_op_requests_queue
        self.assertFalse(workflow_state.station_op_requests_queue)

        station_op = StationOp.from_args()
        workflow_state.station_op_requests_queue.append(station_op)
        self.assertEqual(len(workflow_state.station_op_requests_queue), 1)
        self.assertIsNotNone(workflow_state.station_op_requests_queue.pop())
        self.assertEqual(len(workflow_state.station_op_requests_queue), 0)

        # test proc_requests_queue
        self.assertFalse(workflow_state.proc_requests_queue)

        proc = StationProcess.from_args(lot)
        workflow_state.proc_requests_queue.append(proc)
        self.assertEqual(len(workflow_state.proc_requests_queue), 1)
        self.assertIsNotNone(workflow_state.proc_requests_queue.pop())
        self.assertEqual(len(workflow_state.proc_requests_queue), 0)

    def test_output_state(self):
        output_dict = {
            "location":  {'coordinates': [12, 7], 'descriptor': 'OutputSite'},
            "total_lot_capacity": 2,
            "lot_output_process": None,
            "lots_need_manual_removal": False
        }
        output_state = OutputState.from_dict(output_dict)
        self.assertIsNotNone(output_state.object_id)
        self.assertEqual(output_state.location, Location.from_dict({'coordinates': [12, 7], 'descriptor': 'OutputSite'}))
        self.assertEqual(output_state.total_lot_capacity, 2)
        self.assertEqual(output_state.free_lot_capacity, 2)
        self.assertIsNone(output_state.lot_output_process)
        self.assertFalse(output_state.lots_need_manual_removal)

        # test empty queues
        self.assertFalse(output_state.requested_robot_ops)
        self.assertFalse(output_state.procs_history)

        # test empty lot_slots
        self.assertEqual(len(output_state.lot_slots), 2)
        self.assertIsNone(output_state.lot_slots["0"])
        self.assertIsNone(output_state.lot_slots["1"])
        self.assertEqual(output_state.get_lots_num(), 0)

        # test empty proc slots
        self.assertEqual(len(output_state.proc_slots), 2)
        self.assertIsNone(output_state.proc_slots["0"])
        self.assertIsNone(output_state.proc_slots["1"])

        # test lot_slots
        batch_1 = Batch.from_args(3)
        lot_1 = Lot.from_args([batch_1])
        output_state.lot_slots["0"] = lot_1
        self.assertEqual(output_state.get_lots_num(), 1)
        self.assertEqual(output_state.free_lot_capacity, 1)
        self.assertIsNotNone(output_state.lot_slots["0"])

        batch_2 = Batch.from_args(3)
        lot_2 = Lot.from_args([batch_2])
        output_state.lot_slots["1"] = lot_2
        self.assertEqual(output_state.get_lots_num(), 2)
        self.assertEqual(output_state.free_lot_capacity, 0)
        self.assertIsNotNone(output_state.lot_slots["1"])

        lot_2.status = LotStatus.NEED_REMOVAL
        self.assertEqual(output_state.get_lots_num(status=LotStatus.NEED_REMOVAL), 1)

        # set lots to None
        output_state.lot_slots["1"] = None
        self.assertIsNone(output_state.lot_slots["1"])

        # test requested_robot_ops
        robot_op = RobotOp.from_args()
        output_state.requested_robot_ops.append(robot_op)
        self.assertEqual(len(output_state.requested_robot_ops), 1)
        self.assertIsNotNone(output_state.requested_robot_ops.pop())
        self.assertEqual(len(output_state.requested_robot_ops), 0)

        # test proc_slots
        proc = StationProcess.from_args(lot_1)
        output_state.proc_slots["0"] = proc
        self.assertIsNotNone(output_state.proc_slots["0"])
        output_state.proc_slots["0"] = None
        self.assertIsNone(output_state.proc_slots["0"])

        # test proc history
        output_state.procs_history.append(proc)
        self.assertEqual(len(output_state.procs_history), 1)
        self.assertIsNotNone(output_state.procs_history.pop())
        self.assertEqual(len(output_state.procs_history), 0)


if __name__ == '__main__':
    unittest.main()
