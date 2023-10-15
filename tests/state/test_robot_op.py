import unittest
from datetime import datetime
import uuid
from bson.objectid import ObjectId
from mongoengine import connect

from archemist.core.state.lot import Lot
from archemist.core.state.batch import Batch
from archemist.core.state.robot_op import RobotOpDescriptor, RobotTaskOpDescriptor, RobotTaskType, OpResult, RobotNavOpDescriptor, RobotWaitOpDescriptor
from archemist.core.util.location import Location

class RobotOpTest(unittest.TestCase):
    def setUp(self) -> None:
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

    def  tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_robot_op(self):
        # construct op
        robot_op = RobotOpDescriptor.from_args()
        self.assertIsNotNone(robot_op.uuid)
        self.assertIsNone(robot_op.requested_by)
        self.assertIsNone(robot_op.executed_by)
        self.assertEqual(robot_op.target_robot, "Robot")
        dummy_object_id = ObjectId.from_datetime(datetime.now())
        robot_op.requested_by = dummy_object_id
        self.assertEqual(robot_op.requested_by, dummy_object_id)
        self.assertFalse(robot_op.has_result)
        self.assertIsNone(robot_op.result)
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
        robot_op.complete_op(dummy_robot_object_id, OpResult.SUCCEEDED)
        self.assertTrue(robot_op.has_result)
        self.assertEqual(robot_op.result, OpResult.SUCCEEDED)
        self.assertIsNotNone(robot_op.end_timestamp)
        self.assertLessEqual(robot_op.end_timestamp, datetime.now())
        self.assertGreater(robot_op.end_timestamp, robot_op.start_timestamp)
        end_timestamp = robot_op.end_timestamp
        robot_op.end_timestamp = datetime.now()
        self.assertGreater(robot_op.end_timestamp, end_timestamp)

    def test_robot_task(self):
        # construct op
        station_object_id = ObjectId.from_datetime(datetime.now())
        batch = Batch.from_args(2, Location(1,3,'table_frame'))
        lot = Lot.from_args([batch])
        task_loc = Location(1,3,'table_frame')
        params_dict = {"rack_index": 1, "calibrate": False}
        robot_op = RobotTaskOpDescriptor.from_args("test_task", "TestRobot", RobotTaskType.LOAD_TO_ROBOT,
                                                   params_dict, location=task_loc,
                                                   station_object_id=station_object_id, related_batch=batch)
        self.assertEqual(robot_op._model_proxy._type, "RobotTaskOpDescriptor")
        self.assertEqual(robot_op._model_proxy._module, "archemist.core.state.robot_op")
        self.assertEqual(robot_op.name, "test_task")
        self.assertEqual(robot_op.target_robot, "TestRobot")
        self.assertEqual(robot_op.task_type, RobotTaskType.LOAD_TO_ROBOT)
        self.assertEqual(len(robot_op.params), 2)
        for key, val in params_dict.items():
            self.assertEqual(robot_op.params[key], val)

        self.assertTrue(robot_op.location == task_loc)
        self.assertEqual(robot_op.related_batch, batch)
        self.assertEqual(robot_op.related_lot, lot)

    def test_robot_nav_task(self):
        # construct op
        target_loc = Location(1,3,'')
        params_dict =  {"fine_localisation": True}
        robot_op = RobotNavOpDescriptor.from_args("test_nav_task", "MobileRobot",target_loc, params_dict)
        self.assertIsNotNone(robot_op.uuid)
        self.assertEqual(robot_op.name, "test_nav_task")
        self.assertEqual(robot_op.target_location, target_loc)
        for key, val in params_dict.items():
            self.assertEqual(robot_op.params[key], val)
        self.assertEqual(robot_op._model_proxy._type, "RobotNavOpDescriptor")
        self.assertEqual(robot_op._model_proxy._module, "archemist.core.state.robot_op")

    def test_robot_wait_op(self):
        wait_op = RobotWaitOpDescriptor.from_args(target_robot="Robot", timeout=10)
        self.assertIsNotNone(wait_op.uuid)
        self.assertEqual(wait_op.timeout, 10)

if __name__ == "__main__":
    unittest.main()