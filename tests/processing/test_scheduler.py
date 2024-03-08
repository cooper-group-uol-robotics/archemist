import unittest
from mongoengine import connect
from bson.objectid import ObjectId

from archemist.core.state.robot_op import (RobotTaskOp,
                                           RobotMaintenanceOp,
                                           RobotNavOp,
                                           RobotWaitOp,
                                           CollectBatchOp,
                                           DropBatchOp)
from archemist.core.state.robot import FixedRobot, MobileRobot
from archemist.core.state.state import WorkflowState
from archemist.core.processing.scheduler import PriorityQueueRobotScheduler
from archemist.core.state.station import Station
from archemist.core.state.lot import Batch, Lot
from archemist.core.state.recipe import Recipe
from archemist.core.util.enums import OpOutcome
from archemist.core.util.location import Location


class SchedulerTest(unittest.TestCase):
    def setUp(self) -> None:
        self._db_name = 'archemist_test'
        self._client = connect(
            db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        fixed_robot_dict = {
            "type": "FixedRobot",
            "id": 187,
            "handler": "SimRobotOpHandler"
        }

        self.fixed_robot = FixedRobot.from_dict(fixed_robot_dict)

        mobile_robot_dict = {
            "type": "MobileRobot",
            "id": 137,
            "total_lot_capacity": 1,
            "onboard_capacity": 2,
            "handler": "SimRobotOpHandler"
        }

        self.mobile_robot = MobileRobot.from_dict(mobile_robot_dict)

        station_dict_1 = {
            'type': 'Station',
            'id': 1,
            "location": {"coordinates": [1, 7], "descriptor": "Station1"},
            'handler': 'SimStationOpHandler',
            'total_lot_capacity': 2
        }
        self.station_1 = Station.from_dict(station_dict_1)

        station_dict_2 = {
            'type': 'Station',
            'id': 2,
            "location": {"coordinates": [2, 7], "descriptor": "Station2"},
            'handler': 'SimStationOpHandler',
            'total_lot_capacity': 1
        }
        self.station_2 = Station.from_dict(station_dict_2)

        station_dict_3 = {
            'type': 'Station',
            'id': 3,
            "location": {"coordinates": [3, 7], "descriptor": "Station3"},
            'handler': 'SimStationOpHandler',
            'total_lot_capacity': 1
        }

        self.station_3 = Station.from_dict(station_dict_3)

        station_dict_4 = {
            'type': 'Station',
            'id': 4,
            "location": {"coordinates": [4, 7], "descriptor": "Station4"},
            'handler': 'SimStationOpHandler',
            'total_lot_capacity': 2
        }

        self.station_4 = Station.from_dict(station_dict_4)

        self.workflow_state = WorkflowState.from_args("test_workflow")

        self.recipe_doc_1 = {
            "general": {"name": "test_archemist_recipe", "id": 198},
            "steps": [
                {
                    "state_name": "step_1",
                    "station": {
                        "type": "Station",
                        "id": 1,
                        "process": {
                            "type": "StationProcess",
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
                        "id": 2,
                        "process": {
                            "type": "StationProcess",
                            "operations": None,
                            "args": None,
                        },
                    },
                    "transitions": {
                        "on_success": "step_3",
                        "on_fail": "failed_state",
                    },
                },
                {
                    "state_name": "step_3",
                    "station": {
                        "type": "Station",
                        "id": 3,
                        "process": {
                            "type": "StationProcess",
                            "operations": None,
                            "args": None,
                        },
                    },
                    "transitions": {
                        "on_success": "step_4",
                        "on_fail": "failed_state",
                    },
                },
                {
                    "state_name": "step_4",
                    "station": {
                        "type": "Station",
                        "id": 4,
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

        self.recipe_doc_2 = {
            "general": {"name": "test_archemist_recipe", "id": 199},
            "steps": [
                {
                    "state_name": "step_1",
                    "station": {
                        "type": "Station",
                        "id": 1,
                        "process": {
                            "type": "StationProcess",
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
                        "id": 2,
                        "process": {
                            "type": "StationProcess",
                            "operations": None,
                            "args": None,
                        },
                    },
                    "transitions": {
                        "on_success": "step_3",
                        "on_fail": "failed_state",
                    },
                },
                {
                    "state_name": "step_3",
                    "station": {
                        "type": "Station",
                        "id": 3,
                        "process": {
                            "type": "StationProcess",
                            "operations": None,
                            "args": None,
                        },
                    },
                    "transitions": {
                        "on_success": "step_4",
                        "on_fail": "failed_state",
                    },
                },
                {
                    "state_name": "step_4",
                    "station": {
                        "type": "Station",
                        "id": 4,
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

    def test_priority_queue_scheduler_fixed_robot(self):
        scheduler = PriorityQueueRobotScheduler()
        self.assertDictEqual(scheduler.robots_schedules, {
                             "FixedRobot": [], "MobileRobot": []})
        requester_object_id = ObjectId()

        # create ops queue

        op_1 = RobotTaskOp.from_args("load_rack", "FixedRobot", {"index": 0},
                                     requested_by=requester_object_id)
        op_2 = RobotTaskOp.from_args("load_rack", "FixedRobot", {"index": 1},
                                     requested_by=requester_object_id)
        op_3 = RobotTaskOp.from_args("load_rack", "FixedRobot", {"index": 0})
        self.workflow_state.robot_ops_queue.extend([op_1, op_2, op_3])

        # schedule ops -> robot_queue should be [op_1, op_2]
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 1)
        self.assertEqual(len(self.fixed_robot.queued_ops), 2)
        self.assertEqual(self.fixed_robot.queued_ops[0], op_1)
        self.fixed_robot.update_assigned_op()

        # schedule ops -> op_3 not added since robot attending to requester
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 1)

        # complete the assigned op -> op_1
        self.fixed_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # schedule ops -> op_3 not added since robot attending to requester
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 1)

        op_4 = RobotTaskOp.from_args("unload_rack", "FixedRobot", {"index": 0},
                                     requested_by=requester_object_id)

        # update the assigned op -> op_2
        self.fixed_robot.update_assigned_op()
        self.assertEqual(len(self.fixed_robot.queued_ops), 0)

        # add a new op from the requester
        self.workflow_state.robot_ops_queue.append(op_4)

        # schedule ops -> robot_queue should be [op_4]
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 1)
        self.assertEqual(len(self.fixed_robot.queued_ops), 1)
        self.assertEqual(self.fixed_robot.queued_ops[0], op_4)

        # complete the assigned op -> op_2
        self.fixed_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # upate the assigned op -> op_4
        self.fixed_robot.update_assigned_op()

        # add a maintenance op
        op_5 = RobotMaintenanceOp.from_args("calibrate", "FixedRobot", 187)
        self.workflow_state.robot_ops_queue.append(op_5)

        # schedule ops -> robot_queue should be [op_5], while req_robot_queue be [op_3]
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 1)
        self.assertEqual(len(self.fixed_robot.queued_ops), 1)
        self.assertEqual(self.fixed_robot.queued_ops[0], op_5)

        # complete the assigned op -> op_4
        self.fixed_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # update the assigned op and complete it -> op_5
        self.fixed_robot.update_assigned_op()
        self.fixed_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # schedule ops -> robot_queue should be [op_3]
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertFalse(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.fixed_robot.queued_ops), 1)
        self.assertEqual(self.fixed_robot.queued_ops[0], op_3)

        # update the assigned op and complete it
        self.fixed_robot.update_assigned_op()
        self.fixed_robot.complete_assigned_op(OpOutcome.SUCCEEDED)
        self.assertFalse(self.fixed_robot.queued_ops)
        self.assertIsNone(self.fixed_robot.assigned_op)

    def test_priority_queue_scheduler_mobile_robot(self):
        # construct recipe and lot
        batch_1 = Batch.from_args(3, Location.from_args(
            coordinates=(1, 2), descriptor="some_frame"))
        batch_2 = Batch.from_args(3, Location.from_args(
            coordinates=(1, 2), descriptor="some_frame"))
        lot_1 = Lot.from_args([batch_1, batch_2])
        recipe_1 = Recipe.from_dict(self.recipe_doc_1)
        lot_1.attach_recipe(recipe_1)

        batch_3 = Batch.from_args(3, Location.from_args(
            coordinates=(1, 2), descriptor="some_frame"))
        batch_4 = Batch.from_args(3, Location.from_args(
            coordinates=(1, 2), descriptor="some_frame"))
        lot_2 = Lot.from_args([batch_3, batch_4])
        recipe_2 = Recipe.from_dict(self.recipe_doc_2)
        lot_2.attach_recipe(recipe_2)

        # construct scheduler
        scheduler = PriorityQueueRobotScheduler()
        self.assertDictEqual(scheduler.robots_schedules, {
                             "FixedRobot": [], "MobileRobot": []})

        # add lots to station 1
        self.station_1.add_lot(lot_1)
        self.station_1.add_lot(lot_2)

        # create ops queue
        s1_load_op_1 = CollectBatchOp.from_args("load_rack", "MobileRobot", {"index": 0},
                                                requested_by=self.station_1.object_id,
                                                target_batch=batch_1)
        s1_load_op_2 = CollectBatchOp.from_args("load_rack", "MobileRobot", {"index": 1},
                                                requested_by=self.station_1.object_id,
                                                target_batch=batch_2)
        s1_load_op_3 = CollectBatchOp.from_args("load_rack", "MobileRobot", {"index": 3},
                                                requested_by=self.station_1.object_id,
                                                target_batch=batch_3)
        s1_load_op_4 = CollectBatchOp.from_args("load_rack", "MobileRobot", {"index": 4},
                                                requested_by=self.station_1.object_id,
                                                target_batch=batch_4)

        self.workflow_state.robot_ops_queue.append(s1_load_op_1)
        self.workflow_state.robot_ops_queue.append(s1_load_op_2)
        self.workflow_state.robot_ops_queue.append(s1_load_op_3)
        self.workflow_state.robot_ops_queue.append(s1_load_op_4)

        # schedule ops -> robot_queue should be [s1_load_op_1, s1_load_op_2]
        # while req_robot_queue be [s1_load_op_3, s1_load_op_4]
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 2)
        self.assertEqual(len(self.mobile_robot.queued_ops), 2)
        self.assertEqual(self.mobile_robot.queued_ops[0], s1_load_op_1)

        # update the assigned op and complete it -> s1_load_op_1
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # schedule ops -> no change
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 2)

        # update the assigned op and complete it -> s1_load_op_2
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # complete lot processing for station 1 and remove lot
        self.station_1.finish_processing_lot(lot_1)
        self.station_1.retrieve_ready_for_collection_lots()

        # advance lot recipe and assign lot to station 2
        lot_1.recipe.advance_state(success=True)
        self.station_2.add_lot(lot_1)

        # schedule ops -> no change since robot has no deck space
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 2)

        # add s2 robot ops
        s2_nav_op_1 = RobotNavOp.from_args("move_to_s2", "MobileRobot",
                                           Location.from_args(coordinates=(1, 1)), {
                                               "fine_local": False},
                                           requested_by=self.station_2.object_id,)
        s2_wait_op_1 = RobotWaitOp.from_args("MobileRobot", 5,
                                             requested_by=self.station_2.object_id)

        self.workflow_state.robot_ops_queue.append(s2_nav_op_1)
        self.workflow_state.robot_ops_queue.append(s2_wait_op_1)

        # schedule ops -> robot_queue should be [s2_nav_op_1, s2_wait_op_1]
        # while req_robot_queue be [s1_load_op_3, s1_load_op_4]
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 2)
        self.assertEqual(len(self.mobile_robot.queued_ops), 2)
        self.assertEqual(self.mobile_robot.queued_ops[0], s2_nav_op_1)

        # update the assigned op and complete it -> s2_nav_op_1
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # update the assigned op and complete it -> s2_wait_op_1
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(
            OpOutcome.SUCCEEDED, clear_assigned_op=False)

        # schedule ops -> no change since robot is waiting and also has no space
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 2)

        # add s2 robot ops
        s2_unload_op_1 = DropBatchOp.from_args("unload_rack", "MobileRobot", {"index": 0},
                                               requested_by=self.station_2.object_id,
                                               target_batch=batch_1)
        s2_unload_op_2 = DropBatchOp.from_args("unload_rack", "MobileRobot", {"index": 1},
                                               requested_by=self.station_2.object_id,
                                               target_batch=batch_2)

        self.workflow_state.robot_ops_queue.append(s2_unload_op_1)
        self.workflow_state.robot_ops_queue.append(s2_unload_op_2)

        # schedule ops -> robot_queue should be [s2_unload_op_1, s2_unload_op_2]
        # while req_robot_queue be [s1_load_op_3, s1_load_op_4]
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 2)
        self.assertEqual(len(self.mobile_robot.queued_ops), 2)
        self.assertEqual(self.mobile_robot.queued_ops[0], s2_unload_op_1)

        # clear wait operation
        self.mobile_robot.clear_assigned_op()

        # update the assigned op and complete it -> s2_unload_op_1
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # schedule ops -> no change since robot is assigned to s1
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 2)

        # update the assigned op and complete it -> s2_unload_op_2
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # schedule ops -> no change since s2 is full
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 2)

        # add s2 robot ops
        s2_wait_op_2 = RobotWaitOp.from_args("MobileRobot", 5,
                                             requested_by=self.station_2.object_id)

        self.workflow_state.robot_ops_queue.append(s2_wait_op_2)

        # schedule ops -> robot_queue should be [s2_wait_op_2]
        # while req_robot_queue be [s1_load_op_3, s1_load_op_4]
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 2)
        self.assertEqual(len(self.mobile_robot.queued_ops), 1)
        self.assertEqual(self.mobile_robot.queued_ops[0], s2_wait_op_2)

        # update the assigned op and complete it -> s2_wait_op_1
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(
            OpOutcome.SUCCEEDED, clear_assigned_op=False)

        # create ops queue
        s2_load_op_1 = CollectBatchOp.from_args("load_rack", "MobileRobot", {"index": 0},
                                                requested_by=self.station_2.object_id,
                                                target_batch=batch_1)
        s2_load_op_2 = CollectBatchOp.from_args("load_rack", "MobileRobot", {"index": 1},
                                                requested_by=self.station_2.object_id,
                                                target_batch=batch_2)

        self.workflow_state.robot_ops_queue.append(s2_load_op_1)
        self.workflow_state.robot_ops_queue.append(s2_load_op_2)

        # schedule ops -> robot_queue should be [s2_load_op_1, s2_load_op_2]
        # while req_robot_queue be [s1_load_op_3, s1_load_op_4]
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 2)
        self.assertEqual(len(self.mobile_robot.queued_ops), 2)
        self.assertEqual(self.mobile_robot.queued_ops[0], s2_load_op_1)

        # clear wait operation
        self.mobile_robot.clear_assigned_op()

        # update the assigned op and complete it -> s2_load_op_1
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # update the assigned op and complete it -> s2_load_op_2
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # complete lot processing for station 2 and remove lot
        self.station_2.finish_processing_lot(lot_1)
        self.station_2.retrieve_ready_for_collection_lots()

        # advance lot recipe and assign lot to station 3
        lot_1.recipe.advance_state(success=True)
        self.station_3.add_lot(lot_1)

        # schedule ops -> no change since robot has no deck space
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 2)

        # add s3 robot ops
        s3_man_op_1 = RobotTaskOp.from_args("image_vial", "MobileRobot", {"index": 0},
                                            requested_by=self.station_3.object_id,
                                            target_batch=batch_1)
        s3_wait_op_1 = RobotWaitOp.from_args("MobileRobot", 5,
                                             requested_by=self.station_3.object_id)

        self.workflow_state.robot_ops_queue.append(s3_man_op_1)
        self.workflow_state.robot_ops_queue.append(s3_wait_op_1)

        # schedule ops -> robot_queue should be [s3_man_op_1, s3_wait_op_1]
        # while req_robot_queue be [s1_load_op_3, s1_load_op_4]
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 2)
        self.assertEqual(len(self.mobile_robot.queued_ops), 2)
        self.assertEqual(self.mobile_robot.queued_ops[0], s3_man_op_1)

        # update the assigned op and complete it -> s3_man_op_1
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # update the assigned op and complete it -> s3_wait_op_1
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(
            OpOutcome.SUCCEEDED, clear_assigned_op=False)

        # add s3 robot ops
        s3_unload_op_1 = DropBatchOp.from_args("unload_rack", "MobileRobot", {"index": 0},
                                               requested_by=self.station_3.object_id,
                                               target_batch=batch_1)
        s3_unload_op_2 = DropBatchOp.from_args("unload_rack", "MobileRobot", {"index": 1},
                                               requested_by=self.station_3.object_id,
                                               target_batch=batch_2)

        self.workflow_state.robot_ops_queue.append(s3_unload_op_1)
        self.workflow_state.robot_ops_queue.append(s3_unload_op_2)

        # schedule ops -> robot_queue should be [s3_unload_op_1, s3_unload_op_2]
        # while req_robot_queue be [s1_load_op_3, s1_load_op_4]
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 2)
        self.assertEqual(len(self.mobile_robot.queued_ops), 2)
        self.assertEqual(self.mobile_robot.queued_ops[0], s3_unload_op_1)

        # clear wait operation
        self.mobile_robot.clear_assigned_op()

        # update the assigned op and complete it -> s3_unload_op_1
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # update the assigned op and complete it -> s3_unload_op_2
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # schedule ops -> robot_queue should be [s1_load_op_3, s1_load_op_4]
        # while req_robot_queue be []
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 0)
        self.assertEqual(len(self.mobile_robot.queued_ops), 2)
        self.assertEqual(self.mobile_robot.queued_ops[0], s1_load_op_3)

        # update the assigned op and complete it -> s1_load_op_3
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # update the assigned op and complete it -> s1_load_op_4
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # complete lot processing for station 1 and remove lot
        self.station_1.finish_processing_lot(lot_2)
        self.station_1.retrieve_ready_for_collection_lots()

        # advance lot recipe and assign lot to station 2
        lot_2.recipe.advance_state(success=True)
        self.station_2.add_lot(lot_2)

        # add s2 robot ops
        s2_nav_op_1 = RobotNavOp.from_args("move_to_s2", "MobileRobot",
                                           Location.from_args(coordinates=(1, 1)), {
                                               "fine_local": False},
                                           requested_by=self.station_2.object_id,)
        s2_wait_op_1 = RobotWaitOp.from_args("MobileRobot", 5,
                                             requested_by=self.station_2.object_id)

        self.workflow_state.robot_ops_queue.append(s2_nav_op_1)
        self.workflow_state.robot_ops_queue.append(s2_wait_op_1)

        # schedule ops -> robot_queue should be [s2_nav_op_1, s2_wait_op_1]
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 0)
        self.assertEqual(len(self.mobile_robot.queued_ops), 2)
        self.assertEqual(self.mobile_robot.queued_ops[0], s2_nav_op_1)

        # update the assigned op and complete it -> s2_nav_op_1
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # update the assigned op and complete it -> s2_wait_op_1
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(
            OpOutcome.SUCCEEDED, clear_assigned_op=False)

        # add s2 robot ops
        s2_unload_op_1 = DropBatchOp.from_args("unload_rack", "MobileRobot", {"index": 0},
                                               requested_by=self.station_2.object_id,
                                               target_batch=batch_3)
        s2_unload_op_2 = DropBatchOp.from_args("unload_rack", "MobileRobot", {"index": 1},
                                               requested_by=self.station_2.object_id,
                                               target_batch=batch_4)

        self.workflow_state.robot_ops_queue.append(s2_unload_op_1)
        self.workflow_state.robot_ops_queue.append(s2_unload_op_2)

        # schedule ops -> robot_queue should be [s2_unload_op_1, s2_unload_op_2]
        # while req_robot_queue be [s1_load_op_3, s1_load_op_4]
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 0)
        self.assertEqual(len(self.mobile_robot.queued_ops), 2)
        self.assertEqual(self.mobile_robot.queued_ops[0], s2_unload_op_1)

        # clear wait operation
        self.mobile_robot.clear_assigned_op()

        # update the assigned op and complete it -> s2_unload_op_1
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # update the assigned op and complete it -> s2_unload_op_2
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # schedule ops -> no change since queue is empty
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 0)

        # add s2 ops
        s2_wait_op_2 = RobotWaitOp.from_args("MobileRobot", 5,
                                             requested_by=self.station_2.object_id)

        self.workflow_state.robot_ops_queue.append(s2_wait_op_2)

        # schedule ops -> robot_queue should be [s2_wait_op_2]
        # while req_robot_queue be []
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 0)
        self.assertEqual(len(self.mobile_robot.queued_ops), 1)
        self.assertEqual(self.mobile_robot.queued_ops[0], s2_wait_op_2)

        # update the assigned op and complete it -> s2_wait_op_1
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(
            OpOutcome.SUCCEEDED, clear_assigned_op=False)

        # add s2 ops
        s2_load_op_1 = CollectBatchOp.from_args("load_rack", "MobileRobot", {"index": 0},
                                                requested_by=self.station_2.object_id,
                                                target_batch=batch_3)
        s2_load_op_2 = CollectBatchOp.from_args("load_rack", "MobileRobot", {"index": 1},
                                                requested_by=self.station_2.object_id,
                                                target_batch=batch_4)

        self.workflow_state.robot_ops_queue.append(s2_load_op_1)
        self.workflow_state.robot_ops_queue.append(s2_load_op_2)

        # add s3 ops
        s3_load_op_1 = CollectBatchOp.from_args("load_rack", "MobileRobot", {"index": 0},
                                                requested_by=self.station_3.object_id,
                                                target_batch=batch_1)
        s3_load_op_2 = CollectBatchOp.from_args("load_rack", "MobileRobot", {"index": 1},
                                                requested_by=self.station_3.object_id,
                                                target_batch=batch_2)

        self.workflow_state.robot_ops_queue.append(s3_load_op_1)
        self.workflow_state.robot_ops_queue.append(s3_load_op_2)

        # schedule ops -> robot_queue should be [] since robot is waiting on s2
        # while req_robot_queue be [s2_load_op_1, s2_load_op_2, s3_load_op_1, s3_load_op_2]
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 4)

        # clear wait operation
        self.mobile_robot.clear_assigned_op()

        # schedule ops -> robot_queue should be [s3_load_op_1, s3_load_op_2]
        # while req_robot_queue be [s2_load_op_1, s2_load_op_2] since s3 is full
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 2)
        self.assertEqual(len(self.mobile_robot.queued_ops), 2)
        self.assertEqual(self.mobile_robot.queued_ops[0], s3_load_op_1)

        # update the assigned op and complete it -> s3_load_op_1
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # update the assigned op and complete it -> s3_load_op_2
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # complete lot processing for station 3 and remove lot
        self.station_3.finish_processing_lot(lot_1)
        self.station_3.retrieve_ready_for_collection_lots()

        # advance lot recipe and assign lot to station 4
        lot_1.recipe.advance_state(success=True)
        self.station_4.add_lot(lot_1)

        # add s4 robot ops
        s4_unload_op_1 = DropBatchOp.from_args("unload_rack", "MobileRobot", {"index": 0},
                                               requested_by=self.station_4.object_id,
                                               target_batch=batch_1)
        s4_unload_op_2 = DropBatchOp.from_args("unload_rack", "MobileRobot", {"index": 1},
                                               requested_by=self.station_4.object_id,
                                               target_batch=batch_2)

        self.workflow_state.robot_ops_queue.append(s4_unload_op_1)
        self.workflow_state.robot_ops_queue.append(s4_unload_op_2)

        # schedule ops -> robot_queue should be [s4_unload_op_1, s4_unload_op_2]
        # while req_robot_queue be [s2_load_op_1, s2_load_op_2] since s3 is full
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 2)
        self.assertEqual(len(self.mobile_robot.queued_ops), 2)
        self.assertEqual(self.mobile_robot.queued_ops[0], s4_unload_op_1)

        # update the assigned op and complete it -> s4_unload_op_1
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # update the assigned op and complete it -> s4_unload_op_2
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # complete lot processing for station 3 and remove lot
        self.station_4.finish_processing_lot(lot_1)
        self.station_4.retrieve_ready_for_collection_lots()

        # advance lot recipe and assign lot to station 4
        lot_1.recipe.advance_state(success=True)

        # schedule ops -> robot_queue should be [s2_load_op_1, s2_load_op_2]
        # while req_robot_queue be []
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 0)
        self.assertEqual(len(self.mobile_robot.queued_ops), 2)
        self.assertEqual(self.mobile_robot.queued_ops[0], s2_load_op_1)

        # update the assigned op and complete it -> s2_load_op_1
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # update the assigned op and complete it -> s2_load_op_2
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # complete lot processing for station 2 and remove lot
        self.station_2.finish_processing_lot(lot_2)
        self.station_2.retrieve_ready_for_collection_lots()

        # advance lot recipe and assign lot to station 3
        lot_2.recipe.advance_state(success=True)
        self.station_3.add_lot(lot_2)

        # add s3 robot ops
        s3_man_op_1 = RobotTaskOp.from_args("image_vial", "MobileRobot", {"index": 0},
                                            requested_by=self.station_3.object_id,
                                            target_batch=batch_3)
        s3_wait_op_1 = RobotWaitOp.from_args("MobileRobot", 5,
                                             requested_by=self.station_3.object_id)

        self.workflow_state.robot_ops_queue.append(s3_man_op_1)
        self.workflow_state.robot_ops_queue.append(s3_wait_op_1)

        # schedule ops -> robot_queue should be [s3_man_op_1, s3_wait_op_1]
        # while req_robot_queue be []
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 0)
        self.assertEqual(len(self.mobile_robot.queued_ops), 2)
        self.assertEqual(self.mobile_robot.queued_ops[0], s3_man_op_1)

        # update the assigned op and complete it -> s3_man_op_1
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # update the assigned op and complete it -> s3_wait_op_1
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(
            OpOutcome.SUCCEEDED, clear_assigned_op=False)

        # add s3 robot ops
        s3_unload_op_1 = DropBatchOp.from_args("unload_rack", "MobileRobot", {"index": 0},
                                               requested_by=self.station_3.object_id,
                                               target_batch=batch_3)
        s3_unload_op_2 = DropBatchOp.from_args("unload_rack", "MobileRobot", {"index": 1},
                                               requested_by=self.station_3.object_id,
                                               target_batch=batch_4)

        self.workflow_state.robot_ops_queue.append(s3_unload_op_1)
        self.workflow_state.robot_ops_queue.append(s3_unload_op_2)

        # schedule ops -> robot_queue should be [s3_unload_op_1, s3_unload_op_2]
        # while req_robot_queue be []
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 0)
        self.assertEqual(len(self.mobile_robot.queued_ops), 2)
        self.assertEqual(self.mobile_robot.queued_ops[0], s3_unload_op_1)

        # clear wait operation
        self.mobile_robot.clear_assigned_op()

        # update the assigned op and complete it -> s3_unload_op_1
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # update the assigned op and complete it -> s3_unload_op_2
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # add s3 ops
        s3_load_op_1 = CollectBatchOp.from_args("load_rack", "MobileRobot", {"index": 0},
                                                requested_by=self.station_3.object_id,
                                                target_batch=batch_3)
        s3_load_op_2 = CollectBatchOp.from_args("load_rack", "MobileRobot", {"index": 1},
                                                requested_by=self.station_3.object_id,
                                                target_batch=batch_4)

        self.workflow_state.robot_ops_queue.append(s3_load_op_1)
        self.workflow_state.robot_ops_queue.append(s3_load_op_2)

        # schedule ops -> robot_queue should be [s3_load_op_1, s3_load_op_2]
        # while req_robot_queue be []
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 0)
        self.assertEqual(len(self.mobile_robot.queued_ops), 2)
        self.assertEqual(self.mobile_robot.queued_ops[0], s3_load_op_1)

        # update the assigned op and complete it -> s3_load_op_1
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # update the assigned op and complete it -> s3_load_op_1
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # complete lot processing for station 3 and remove lot
        self.station_3.finish_processing_lot(lot_2)
        self.station_3.retrieve_ready_for_collection_lots()

        # advance lot recipe and assign lot to station 4
        lot_2.recipe.advance_state(success=True)
        self.station_4.add_lot(lot_2)

        # add s4 robot ops
        s4_unload_op_1 = DropBatchOp.from_args("unload_rack", "MobileRobot", {"index": 0},
                                               requested_by=self.station_4.object_id,
                                               target_batch=batch_3)
        s4_unload_op_2 = DropBatchOp.from_args("unload_rack", "MobileRobot", {"index": 1},
                                               requested_by=self.station_4.object_id,
                                               target_batch=batch_4)

        self.workflow_state.robot_ops_queue.append(s4_unload_op_1)
        self.workflow_state.robot_ops_queue.append(s4_unload_op_2)

        # schedule ops -> robot_queue should be [s4_unload_op_1, s4_unload_op_2]
        # while req_robot_queue be []
        scheduler.schedule(self.workflow_state.robot_ops_queue)
        self.assertEqual(len(self.workflow_state.robot_ops_queue), 0)
        self.assertEqual(len(self.mobile_robot.queued_ops), 2)
        self.assertEqual(self.mobile_robot.queued_ops[0], s4_unload_op_1)

        # update the assigned op and complete it -> s4_unload_op_1
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # update the assigned op and complete it -> s4_unload_op_2
        self.mobile_robot.update_assigned_op()
        self.mobile_robot.complete_assigned_op(OpOutcome.SUCCEEDED)

        # complete lot processing for station 3 and remove lot
        self.station_4.finish_processing_lot(lot_2)
        self.station_4.retrieve_ready_for_collection_lots()

        # advance lot recipe and assign lot to station 4
        lot_2.recipe.advance_state(success=True)


if __name__ == "__main__":
    unittest.main()
