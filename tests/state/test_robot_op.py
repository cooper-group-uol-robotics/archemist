import unittest
from datetime import datetime
from bson.objectid import ObjectId
from mongoengine import connect

from archemist.core.state.lot import Lot
from archemist.core.state.batch import Batch
from archemist.core.state.robot_op import (RobotOp,
                                           RobotTaskOp,
                                           CollectBatchOp,
                                           DropBatchOp,
                                           OpOutcome,
                                           RobotNavOp,
                                           RobotWaitOp)
from archemist.core.util.location import Location


class RobotOpTest(unittest.TestCase):
    def setUp(self) -> None:
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

    def tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_robot_op(self):
        # construct op
        robot_op = RobotOp.from_args()
        self.assertIsNotNone(robot_op.object_id)
        self.assertIsNone(robot_op.requested_by)
        self.assertIsNone(robot_op.executed_by)
        self.assertEqual(robot_op.target_robot, "Robot")
        dummy_object_id = ObjectId.from_datetime(datetime.now())
        robot_op.requested_by = dummy_object_id
        self.assertEqual(robot_op.requested_by, dummy_object_id)
        self.assertIsNone(robot_op.outcome)
        self.assertIsNone(robot_op.start_timestamp)
        self.assertIsNone(robot_op.end_timestamp)

        # test start timestamp
        robot_op.add_start_timestamp()
        self.assertIsNotNone(robot_op.start_timestamp)
        self.assertLessEqual(robot_op.start_timestamp, datetime.now())
        start_timestamp = robot_op.start_timestamp
        robot_op.start_timestamp = datetime.now()
        self.assertGreater(robot_op.start_timestamp, start_timestamp)

        # test end timestamp
        dummy_robot_object_id = ObjectId.from_datetime(datetime.now())
        robot_op.complete_op(dummy_robot_object_id, OpOutcome.SUCCEEDED)
        self.assertEqual(robot_op.outcome, OpOutcome.SUCCEEDED)
        self.assertIsNotNone(robot_op.end_timestamp)
        self.assertLessEqual(robot_op.end_timestamp, datetime.now())
        self.assertGreater(robot_op.end_timestamp, robot_op.start_timestamp)
        end_timestamp = robot_op.end_timestamp
        robot_op.end_timestamp = datetime.now()
        self.assertGreater(robot_op.end_timestamp, end_timestamp)

    def test_robot_task(self):
        # construct op
        requested_by = ObjectId.from_datetime(datetime.now())
        batch = Batch.from_args(2)
        lot = Lot.from_args([batch])
        task_loc = Location.from_args(coordinates=(1, 3), descriptor="ChemspeedStation")
        params_dict = {"rack_index": 1, "calibrate": False}
        robot_op = RobotTaskOp.from_args("test_task", "TestRobot",
                                         params_dict, target_location=task_loc,
                                         requested_by=requested_by,
                                         target_batch=batch)
        self.assertEqual(robot_op._model_proxy._type, "RobotTaskOp")
        self.assertEqual(robot_op._model_proxy._module, "archemist.core.state.robot_op")
        self.assertEqual(robot_op.name, "test_task")
        self.assertEqual(robot_op.target_robot, "TestRobot")
        self.assertEqual(len(robot_op.params), 2)
        for key, val in params_dict.items():
            self.assertEqual(robot_op.params[key], val)

        self.assertEqual(robot_op.target_location, task_loc)
        self.assertEqual(robot_op.target_batch, batch)
        self.assertEqual(robot_op.related_lot, lot)

        # test op with unspecified location
        no_location_robot_op = RobotTaskOp.from_args("test_task", "TestRobot",
                                                     params_dict,
                                                     requested_by=requested_by,
                                                     target_batch=batch)
        self.assertTrue(no_location_robot_op.target_location.is_unspecified())
        no_location_robot_op.target_location = task_loc
        self.assertEqual(no_location_robot_op.target_location, task_loc)

    def test_collect_batch_task(self):
        # construct op
        requested_by = ObjectId.from_datetime(datetime.now())
        batch = Batch.from_args(2)
        lot = Lot.from_args([batch])
        task_loc = Location.from_args(coordinates=(2, 3), descriptor="InputSite")
        params_dict = {"rack_index": 1, "calibrate": False}
        robot_op = CollectBatchOp.from_args("test_task", "TestRobot",
                                            params_dict, target_location=task_loc,
                                            requested_by=requested_by,
                                            target_batch=batch)
        self.assertEqual(robot_op._model_proxy._type, "CollectBatchOp")
        self.assertEqual(robot_op._model_proxy._module, "archemist.core.state.robot_op")
        self.assertEqual(robot_op.name, "test_task")
        self.assertEqual(robot_op.target_robot, "TestRobot")
        self.assertEqual(len(robot_op.params), 2)
        for key, val in params_dict.items():
            self.assertEqual(robot_op.params[key], val)

        self.assertTrue(robot_op.target_location == task_loc)
        self.assertEqual(robot_op.target_batch, batch)
        self.assertEqual(robot_op.related_lot, lot)
        self.assertIsNone(robot_op.target_onboard_slot)
        robot_op.target_onboard_slot = 1
        self.assertEqual(robot_op.target_onboard_slot, 1)

    def test_drop_batch_task(self):
        # construct op
        requested_by = ObjectId.from_datetime(datetime.now())
        batch = Batch.from_args(2)
        lot = Lot.from_args([batch])
        task_loc = Location.from_args(coordinates=(2, 3), descriptor="OutputSite")
        params_dict = {"rack_index": 1, "calibrate": False}
        robot_op = DropBatchOp.from_args("test_task", "TestRobot",
                                         params_dict, target_location=task_loc,
                                         requested_by=requested_by,
                                         target_batch=batch)
        self.assertEqual(robot_op.name, "test_task")
        self.assertEqual(robot_op.target_robot, "TestRobot")
        self.assertEqual(len(robot_op.params), 2)
        for key, val in params_dict.items():
            self.assertEqual(robot_op.params[key], val)

        self.assertTrue(robot_op.target_location == task_loc)
        self.assertEqual(robot_op.target_batch, batch)
        self.assertEqual(robot_op.related_lot, lot)
        self.assertIsNone(robot_op.onboard_collection_slot)
        robot_op.onboard_collection_slot = 1
        self.assertEqual(robot_op.onboard_collection_slot, 1)

    def test_robot_nav_task(self):
        # construct op
        target_loc = Location.from_args(coordinates=(2, 3), descriptor="InputSite")
        params_dict = {"fine_localisation": True}
        robot_op = RobotNavOp.from_args("test_nav_task", "MobileRobot", target_loc, params_dict)
        self.assertIsNotNone(robot_op.object_id)
        self.assertEqual(robot_op.name, "test_nav_task")
        self.assertEqual(robot_op.target_location, target_loc)
        for key, val in params_dict.items():
            self.assertEqual(robot_op.params[key], val)
        self.assertEqual(robot_op._model_proxy._type, "RobotNavOp")
        self.assertEqual(robot_op._model_proxy._module, "archemist.core.state.robot_op")

    def test_robot_wait_op(self):
        wait_op = RobotWaitOp.from_args(target_robot="Robot", timeout=10)
        self.assertIsNotNone(wait_op.object_id)
        self.assertEqual(wait_op.timeout, 10)


if __name__ == "__main__":
    unittest.main()
