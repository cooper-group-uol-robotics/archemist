from datetime import datetime
from bson.objectid import ObjectId
import unittest
from archemist.core.state.robot import Robot, MobileRobot, RobotState, MobileRobotMode, OpState, OpOutcome
from archemist.core.state.lot import Lot
from archemist.core.state.batch import Batch
from archemist.core.state.robot_op import RobotTaskOp, CollectBatchOp, DropBatchOp
from archemist.core.util.location import Location
from mongoengine import connect


class RobotTest(unittest.TestCase):
    def setUp(self) -> None:
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

    def tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_robot(self):
        robot_dict = {
            "type": "Robot",
            "id": 187,
            "handler": "SimRobotOpHandler"
        }

        # test general fields
        robot = Robot.from_dict(robot_dict)
        self.assertEqual(robot.id, 187)
        self.assertEqual(robot.selected_handler, "SimRobotOpHandler")
        self.assertEqual(robot.module_path, "archemist.core.state.robot")
        self.assertEqual(robot.location, Location.from_args(coordinates=(), descriptor="unknown"))
        t_loc = Location.from_args(coordinates=(1, 2), descriptor="InputSite")
        robot.location = t_loc
        self.assertEqual(robot.location, t_loc)
        self.assertIsNone(robot.attending_to)
        self.assertEqual(robot.state, RobotState.INACTIVE)
        robot.state = RobotState.ACTIVE
        self.assertEqual(robot.state, RobotState.ACTIVE)
        self.assertEqual(len(robot.ops_history), 0)

        # test op operation
        # construct op
        requested_by = ObjectId.from_datetime(datetime.now())
        task_loc = Location.from_args(coordinates=(1, 3), descriptor='input_site')
        params = {"rack_number": 1, "calibrate": False}
        robot_op_1 = RobotTaskOp.from_args("test_task1", "Robot",
                                           params, target_location=task_loc,
                                           requested_by=requested_by)
        robot_op_2 = RobotTaskOp.from_args("test_task2", "Robot",
                                           params, target_location=task_loc,
                                           requested_by=requested_by)
        # assign op
        self.assertEqual(robot.assigned_op_state, OpState.INVALID)
        self.assertFalse(robot.queued_ops)
        self.assertIsNone(robot.assigned_op)
        robot.add_op(robot_op_1)
        self.assertEqual(len(robot.queued_ops), 1)
        self.assertIsNone(robot.assigned_op)
        robot.add_op(robot_op_2)
        self.assertEqual(len(robot.queued_ops), 2)

        # update assigned op
        robot.update_assigned_op()
        self.assertEqual(robot.assigned_op_state, OpState.ASSIGNED)
        self.assertEqual(len(robot.queued_ops), 1)
        self.assertIsNotNone(robot.assigned_op)

        # test no update since assigned op is not none
        robot.update_assigned_op()
        self.assertEqual(robot.assigned_op_state, OpState.ASSIGNED)
        self.assertEqual(len(robot.queued_ops), 1)

        # get assigned op
        assigned_op = robot.assigned_op
        self.assertIsNotNone(assigned_op)
        self.assertEqual(assigned_op, robot_op_1)

        # start executing
        robot.set_assigned_op_to_execute()
        self.assertEqual(robot.assigned_op_state, OpState.EXECUTING)

        # repeat and skip op
        robot.skip_assigned_op()
        self.assertEqual(robot.assigned_op_state, OpState.TO_BE_SKIPPED)
        robot.repeat_assigned_op()
        self.assertEqual(robot.assigned_op_state, OpState.TO_BE_REPEATED)

        # return to start executing
        robot.set_assigned_op_to_execute()
        self.assertEqual(robot.attending_to, requested_by)

        # complete robot job
        robot.complete_assigned_op(OpOutcome.SUCCEEDED)
        self.assertIsNone(robot.assigned_op)
        self.assertEqual(robot.assigned_op_state, OpState.INVALID)
        self.assertEqual(robot.attending_to, requested_by)

        # test history
        history = robot.ops_history
        self.assertEqual(len(history), 1)
        self.assertEqual(history[0], robot_op_1)
        self.assertEqual(history[0].executed_by,  robot.model.id)
        self.assertEqual(history[0].outcome, OpOutcome.SUCCEEDED)

        # update assigned op
        robot.update_assigned_op()
        self.assertEqual(robot.assigned_op_state, OpState.ASSIGNED)
        self.assertFalse(robot.queued_ops)

        # test complete assigned op without clearning slot
        robot.complete_assigned_op(OpOutcome.SUCCEEDED, clear_assigned_op=False)
        self.assertIsNotNone(robot.assigned_op)
        self.assertEqual(robot.assigned_op_state, OpState.ASSIGNED)
        self.assertEqual(len(robot.ops_history), 1)

        # test clear assigned op
        robot.clear_assigned_op()
        self.assertIsNone(robot.assigned_op)
        self.assertEqual(robot.assigned_op_state, OpState.INVALID)
        self.assertIsNone(robot.attending_to)

        # test history
        history = robot.ops_history
        self.assertEqual(len(history), 2)
        self.assertEqual(history[1], robot_op_2)

    def test_mobile_robot(self):
        robot_dict = {
            "type": "MobileRobot",
            "location": {"coordinates": [1, 2], "descriptor": "InputSite"},
            "id": 187,
            "total_lot_capacity": 1,
            "onboard_capacity": 2,
            "handler": "SimRobotOpHandler"
        }

        # test general fields
        robot = MobileRobot.from_dict(robot_dict)
        self.assertEqual(robot.id, 187)
        self.assertEqual(robot.selected_handler, "SimRobotOpHandler")
        self.assertEqual(robot.module_path, "archemist.core.state.robot")
        self.assertEqual(robot.location, Location.from_args(coordinates=(1, 2), descriptor="InputSite"))
        self.assertEqual(robot.operational_mode, MobileRobotMode.OPERATIONAL)
        robot.operational_mode = MobileRobotMode.COOLDOWN
        self.assertEqual(robot.operational_mode, MobileRobotMode.COOLDOWN)
        self.assertEqual(robot.total_lot_capacity, 1)
        self.assertEqual(robot.free_lot_capacity, 1)
        self.assertEqual(robot.onboard_capacity, 2)
        self.assertEqual(robot.free_batch_capacity, 2)
        self.assertIsNone(robot.onboard_batches_slots["0"])
        self.assertIsNone(robot.onboard_batches_slots["1"])
        self.assertFalse(robot.consigned_lots)

        # create batches and loading ops
        batch_1 = Batch.from_args(2, Location.from_args(coordinates=(1, 3), descriptor='table_frame'))
        batch_2 = Batch.from_args(2, Location.from_args(coordinates=(1, 3), descriptor='table_frame'))
        lot = Lot.from_args([batch_1, batch_2])
        task_loc = batch_1.location
        params = {"rack_number": 1, "calibrate": False}
        loading_robot_op_1 = CollectBatchOp.from_args("load_batch", "Robot",
                                                      params, target_location=task_loc, target_batch=batch_1)
        loading_robot_op_2 = CollectBatchOp.from_args("load_batch", "Robot",
                                                      params, target_location=task_loc, target_batch=batch_2)

        # test loading batches
        self.assertEqual(robot.free_batch_capacity, 2)
        robot.add_op(loading_robot_op_1)
        self.assertIsNone(loading_robot_op_1.target_onboard_slot)
        self.assertEqual(len(robot.consigned_lots), 1)
        self.assertEqual(robot.free_lot_capacity, 0)
        self.assertFalse(robot.is_batch_onboard(batch_1))
        self.assertFalse(robot.is_batch_onboard(batch_2))

        robot.update_assigned_op()
        self.assertEqual(loading_robot_op_1.target_onboard_slot, 0)
        robot.complete_assigned_op(OpOutcome.SUCCEEDED)
        self.assertEqual(robot.onboard_batches_slots["0"], batch_1)
        self.assertEqual(robot.free_batch_capacity, 1)
        self.assertTrue(robot.is_batch_onboard(batch_1))
        self.assertEqual(batch_1.location, Location.from_args(descriptor=f"{robot} @ slot:{loading_robot_op_1.target_onboard_slot}"))

        robot.add_op(loading_robot_op_2)
        self.assertIsNone(loading_robot_op_2.target_onboard_slot)
        self.assertEqual(len(robot.consigned_lots), 1)
        robot.update_assigned_op()
        self.assertEqual(loading_robot_op_2.target_onboard_slot, 1)
        robot.complete_assigned_op(OpOutcome.SUCCEEDED)
        self.assertEqual(robot.onboard_batches_slots["1"], batch_2)
        self.assertEqual(robot.free_batch_capacity, 0)
        self.assertTrue(robot.is_batch_onboard(batch_2))

        # create unloading ops
        unloading_robot_op_1 = DropBatchOp.from_args("unload_batch", "Robot",
                                                     params, target_location=task_loc, target_batch=batch_1)
        unloading_robot_op_2 = DropBatchOp.from_args("unload_batch", "Robot",
                                                     params, target_location=task_loc, target_batch=batch_2)

        # test unloading batches
        self.assertEqual(robot.free_batch_capacity, 0)
        robot.add_op(unloading_robot_op_1)
        self.assertIsNone(unloading_robot_op_1.onboard_collection_slot)
        robot.update_assigned_op()
        self.assertEqual(unloading_robot_op_1.onboard_collection_slot, 0)
        robot.complete_assigned_op(OpOutcome.SUCCEEDED)
        self.assertEqual(len(robot.consigned_lots), 1)
        self.assertIsNone(robot.onboard_batches_slots["0"])
        self.assertFalse(robot.is_batch_onboard(batch_1))
        self.assertEqual(batch_1.location, task_loc)
        self.assertEqual(robot.free_batch_capacity, 1)

        robot.add_op(unloading_robot_op_2)
        self.assertIsNone(unloading_robot_op_2.onboard_collection_slot)
        robot.update_assigned_op()
        self.assertEqual(unloading_robot_op_2.onboard_collection_slot, 1)
        robot.complete_assigned_op(OpOutcome.SUCCEEDED)
        self.assertEqual(len(robot.consigned_lots), 0)
        self.assertEqual(robot.free_lot_capacity, 1)
        self.assertIsNone(robot.onboard_batches_slots["1"])
        self.assertFalse(robot.is_batch_onboard(batch_2))
        self.assertEqual(robot.free_batch_capacity, 2)


if __name__ == '__main__':
    unittest.main()
