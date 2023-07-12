from datetime import datetime
from bson.objectid import ObjectId
import unittest
from archemist.core.state.robot import Robot, MobileRobot, RobotState,RobotTaskType, MobileRobotMode
from archemist.core.state.batch import Batch
from archemist.core.state.robot_op import RobotTaskOpDescriptor
from archemist.core.util.location import Location
from archemist.core.exceptions.exception import RobotAssignedRackError
from mongoengine import connect


class RobotTest(unittest.TestCase):
    def setUp(self) -> None:
        connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')

    def test_robot(self):
        robot_dict = {
            "type": "SomeRobot",
            "location": {"node_id": 1, "graph_id": 2, "frame_name": "a_frame"},
            "id": 187,
            "batch_capacity":2,
            "handler": "GenericRobotHandler"
        }

        # test general fields
        robot = Robot.from_dict(robot_dict)
        self.assertEqual(robot.id, 187)
        self.assertEqual(robot.selected_handler_dict, {"module": "archemist.core.state", "type": "GenericRobotHandler"})
        self.assertTrue(robot.operational)
        robot.operational = False
        self.assertFalse(robot.operational)
        self.assertEqual(robot.location, Location(node_id=1, graph_id=2, frame_name="a_frame"))
        t_loc = Location(node_id=3, graph_id=2, frame_name="b_frame")
        robot.location = t_loc
        self.assertEqual(robot.location, t_loc)
        self.assertEqual(robot.state, RobotState.IDLE)
        self.assertEqual(len(robot.op_history), 0)

        # test op operation
        # construct op
        station_object_id = ObjectId.from_datetime(datetime.now())
        task_loc = Location(1,3,'table_frame')
        params_list = ["1", "false"]
        robot_op = RobotTaskOpDescriptor.from_args("test_task", RobotTaskType.LOAD_TO_ROBOT,
                                                   params_list, task_loc,
                                                   station_object_id)
        # assign op
        self.assertFalse(robot.has_assigned_op())
        robot.assign_op(robot_op)
        self.assertEqual(robot.state, RobotState.OP_ASSIGNED)
        self.assertTrue(robot.has_assigned_op())

        # get assigned op
        assigned_op = robot.get_assigned_op()
        self.assertIsNotNone(assigned_op)
        self.assertEqual(assigned_op.name, robot_op.name)
        self.assertEqual(assigned_op.location, robot_op.location)
        self.assertEqual(assigned_op.requested_by, robot_op.requested_by)
        for idx, param in enumerate(assigned_op.params):
            self.assertEqual(robot_op.params[idx], param)
        with self.assertRaises(RobotAssignedRackError):
            robot.assign_op(robot_op)
        
        # start executing
        self.assertIsNone(assigned_op.start_timestamp)
        robot.start_executing_op()
        self.assertEqual(robot.state, RobotState.EXECUTING_OP)
        self.assertIsNotNone(assigned_op.start_timestamp)

        # repeat and skip op
        robot.skip_assigned_op()
        self.assertEqual(robot.state, RobotState.SKIP_OP)
        robot.repeat_assigned_op()
        self.assertEqual(robot.state, RobotState.REPEAT_OP)

        # return to start executing
        robot.start_executing_op()
       
        # complete robot job
        self.assertFalse(robot.is_assigned_op_complete())
        robot.complete_assigned_op(True)
        self.assertFalse(robot.has_assigned_op())
        self.assertEqual(robot.state, RobotState.EXECUTION_COMPLETE)
        self.assertTrue(robot.is_assigned_op_complete())

        # retrieve robot job
        ret_op = robot.get_complete_op()
        self.assertEqual(robot.state, RobotState.IDLE)
        self.assertFalse(robot.is_assigned_op_complete())
        # self.assertEqual(len(robot.onboard_batches),1)
        # self.assertEqual(robot.onboard_batches[0],32)
        self.assertEqual(ret_op.name, robot_op.name)
        self.assertEqual(len(ret_op.params), len(robot_op.params))
        self.assertEqual(ret_op.requested_by, robot_op.requested_by)
        self.assertEqual(ret_op.executed_by, robot.model.id)
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)

        history = robot.op_history
        self.assertEqual(len(history), 1)
        self.assertEqual(history[0].name, robot_op.name)
        self.assertEqual(history[0].location, robot_op.location)
        self.assertEqual(history[0].executed_by,  robot.model.id)
        self.assertTrue(history[0].has_result)
        self.assertTrue(history[0].was_successful)

    def test_mobile_robot(self):
        robot_dict = {
            "type": "SomeRobot",
            "location": {"node_id": 1, "graph_id": 2, "frame_name": "a_frame"},
            "id": 187,
            "batch_capacity":2,
            "handler": "GenericRobotHandler"
        }

        # test general fields
        robot = MobileRobot.from_dict(robot_dict)
        self.assertEqual(robot.id, 187)
        self.assertEqual(robot.selected_handler_dict, {"module": "archemist.core.state", "type": "GenericRobotHandler"})
        self.assertTrue(robot.operational)
        self.assertEqual(robot.location, Location(node_id=1, graph_id=2, frame_name="a_frame"))
        self.assertEqual(robot.operational_mode, MobileRobotMode.OPERTIAONAL)
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
        robot.assign_op(loading_robot_op_1)
        robot.complete_assigned_op(True)
        robot.get_complete_op()
        self.assertEqual(len(robot.onboard_batches), 1)
        self.assertTrue(robot.is_batch_onboard(loading_robot_op_1.related_batch))

        robot.assign_op(loading_robot_op_2)
        robot.complete_assigned_op(True)
        robot.get_complete_op()
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
        robot.assign_op(unloading_robot_op_1)
        robot.complete_assigned_op(True)
        robot.get_complete_op()
        self.assertEqual(len(robot.onboard_batches), 1)
        self.assertFalse(robot.is_batch_onboard(loading_robot_op_1.related_batch))

        robot.assign_op(unloading_robot_op_2)
        robot.complete_assigned_op(True)
        robot.get_complete_op()
        self.assertEqual(len(robot.onboard_batches), 0)
        self.assertFalse(robot.is_batch_onboard(loading_robot_op_2.related_batch))
        self.assertFalse(robot.is_onboard_capacity_full())



if __name__ == '__main__':
    unittest.main()

        
        
