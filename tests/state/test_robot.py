from datetime import datetime
from bson.objectid import ObjectId
import unittest
from archemist.core.state.robot import Robot, MobileRobot, RobotState,RobotTaskType, MobileRobotMode, OpState
from archemist.core.state.batch import Batch
from archemist.core.state.robot_op import RobotTaskOpDescriptor
from archemist.core.util.location import Location
from archemist.core.exceptions.exception import RobotAssignedRackError
from mongoengine import connect


class RobotTest(unittest.TestCase):
    def setUp(self) -> None:
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

    def  tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()  

    def test_robot(self):
        robot_dict = {
            "type": "Robot",
            "location": {"node_id": 1, "graph_id": 2, "frame_name": "a_frame"},
            "id": 187,
            "batch_capacity":2,
            "handler": "GenericRobotHandler"
        }

        # test general fields
        robot = Robot.from_dict(robot_dict)
        self.assertEqual(robot.id, 187)
        self.assertEqual(robot.selected_handler, "GenericRobotHandler")
        self.assertEqual(robot.module_path, "archemist.core.state.robot")
        self.assertEqual(robot.location, Location(node_id=1, graph_id=2, frame_name="a_frame"))
        t_loc = Location(node_id=3, graph_id=2, frame_name="b_frame")
        robot.location = t_loc
        self.assertEqual(robot.location, t_loc)
        self.assertEqual(robot.state, RobotState.INACTIVE)
        robot.state = RobotState.ACTIVE
        self.assertEqual(robot.state, RobotState.ACTIVE)
        self.assertEqual(len(robot.ops_history), 0)

        # test op operation
        # construct op
        station_object_id = ObjectId.from_datetime(datetime.now())
        task_loc = Location(1,3,'table_frame')
        params_list = ["1", "false"]
        robot_op = RobotTaskOpDescriptor.from_args("test_task", RobotTaskType.LOAD_TO_ROBOT,
                                                   params_list, task_loc,
                                                   station_object_id)
        # assign op
        self.assertEqual(robot.assigned_op_state, OpState.INVALID)
        self.assertFalse(robot.assigned_op)
        robot.add_op(robot_op)
        self.assertEqual(robot.assigned_op_state, OpState.ASSIGNED)

        # get assigned op
        assigned_op = robot.assigned_op
        self.assertIsNotNone(assigned_op)
        self.assertEqual(assigned_op.name, robot_op.name)
        self.assertEqual(assigned_op.location, robot_op.location)
        self.assertEqual(assigned_op.requested_by, robot_op.requested_by)
        for idx, param in enumerate(assigned_op.params):
            self.assertEqual(robot_op.params[idx], param)
        with self.assertRaises(RobotAssignedRackError):
            robot.add_op(robot_op)
        
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
       
        # complete robot job
        robot.complete_assigned_op(True)
        self.assertIsNone(robot.assigned_op)
        self.assertEqual(robot.assigned_op_state, OpState.INVALID)        

        history = robot.ops_history
        self.assertEqual(len(history), 1)
        self.assertEqual(history[0].name, robot_op.name)
        self.assertEqual(history[0].location, robot_op.location)
        self.assertEqual(history[0].executed_by,  robot.model.id)
        self.assertTrue(history[0].has_result)
        self.assertTrue(history[0].was_successful)

    def test_mobile_robot(self):
        robot_dict = {
            "type": "MobileRobot",
            "location": {"node_id": 1, "graph_id": 2, "frame_name": "a_frame"},
            "id": 187,
            "batch_capacity":2,
            "handler": "GenericRobotHandler"
        }

        # test general fields
        robot = MobileRobot.from_dict(robot_dict)
        self.assertEqual(robot.id, 187)
        self.assertEqual(robot.selected_handler, "GenericRobotHandler")
        self.assertEqual(robot.module_path, "archemist.core.state.robot")
        self.assertEqual(robot.location, Location(node_id=1, graph_id=2, frame_name="a_frame"))
        self.assertEqual(robot.operational_mode, MobileRobotMode.OPERATIONAL)
        robot.operational_mode = MobileRobotMode.COOLDOWN
        self.assertEqual(robot.operational_mode, MobileRobotMode.COOLDOWN)
        self.assertEqual(robot.batch_capacity, 2)
        self.assertEqual(len(robot.onboard_batches), 0)

        # create batches and loading ops
        station_object_id = ObjectId.from_datetime(datetime.now())
        batch_1 = Batch.from_arguments(2, Location(1,3,'table_frame'))
        batch_2 = Batch.from_arguments(2, Location(1,3,'table_frame'))
        task_loc = Location(1,3,'table_frame')
        params_list = ["1", "false"]
        loading_robot_op_1 = RobotTaskOpDescriptor.from_args("load_batch", RobotTaskType.LOAD_TO_ROBOT,
                                                   params_list, task_loc,
                                                   station_object_id, batch_1)
        loading_robot_op_2 = RobotTaskOpDescriptor.from_args("load_batch", RobotTaskType.LOAD_TO_ROBOT,
                                                   params_list, task_loc,
                                                   station_object_id, batch_2)

        # test loading batches
        self.assertFalse(robot.is_onboard_capacity_full())
        robot.add_op(loading_robot_op_1)
        robot.complete_assigned_op(True)
        self.assertEqual(len(robot.onboard_batches), 1)
        self.assertTrue(robot.is_batch_onboard(loading_robot_op_1.related_batch))

        robot.add_op(loading_robot_op_2)
        robot.complete_assigned_op(True)
        self.assertEqual(len(robot.onboard_batches), 2)
        self.assertTrue(robot.is_batch_onboard(loading_robot_op_2.related_batch))

        # create unloading ops
        unloading_robot_op_1 = RobotTaskOpDescriptor.from_args("unload_batch", RobotTaskType.UNLOAD_FROM_ROBOT,
                                                   params_list, task_loc,
                                                   station_object_id, batch_1)
        unloading_robot_op_2 = RobotTaskOpDescriptor.from_args("unload_batch", RobotTaskType.UNLOAD_FROM_ROBOT,
                                                   params_list, task_loc,
                                                   station_object_id, batch_2)

        # test unloading batches
        self.assertTrue(robot.is_onboard_capacity_full())
        robot.add_op(unloading_robot_op_1)
        robot.complete_assigned_op(True)
        self.assertEqual(len(robot.onboard_batches), 1)
        self.assertFalse(robot.is_batch_onboard(loading_robot_op_1.related_batch))

        robot.add_op(unloading_robot_op_2)
        robot.complete_assigned_op(True)
        self.assertEqual(len(robot.onboard_batches), 0)
        self.assertFalse(robot.is_batch_onboard(loading_robot_op_2.related_batch))
        self.assertFalse(robot.is_onboard_capacity_full())


if __name__ == '__main__':
    unittest.main()

        
        
