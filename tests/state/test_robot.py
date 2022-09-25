from bson.objectid import ObjectId
import unittest
from archemist.state.robots import PandaFranka
from archemist.state.robot import RobotState, MoveSampleOp, RobotOutputDescriptor
from archemist.util.location import Location
from archemist.util.station_robot_job import StationRobotJob
from archemist.exceptions.exception import RobotAssignedRackError


class RobotTest(unittest.TestCase):

    def test_robot(self):
        robot_dict = {
            'class': 'PandaFranka',
            'id': 187
        }

        t_robot = PandaFranka.from_dict('test', robot_dict)
        self.assertEqual(t_robot.id, 187)
        self.assertEqual(t_robot.location, None)
        t_robot.location = Location(node_id=1, graph_id=7, frame_name='neutral_frame')
        self.assertEqual(t_robot.location, Location(node_id=1, graph_id=7, frame_name='neutral_frame'))
        self.assertEqual(t_robot.state, RobotState.IDLE)
        self.assertEqual(t_robot.operational, True)

        # Robot job
        # assign job
        self.assertEqual(t_robot.robot_job_history, [])
        start_loc = Location(node_id=1, graph_id=7, frame_name='rackholder_frame')
        end_loc = Location(node_id=1, graph_id=7, frame_name='rackholder_frame')
        robot_job = MoveSampleOp(1,start_loc, end_loc, RobotOutputDescriptor())

        stn_obj_id  = ObjectId('0123456789ab0123456789ab')
        stn_robot_job = StationRobotJob(robot_job, stn_obj_id)
        t_robot.assign_job(stn_robot_job)
        self.assertEqual(t_robot.state, RobotState.EXECUTING_JOB)
        assigned_job = t_robot.assigned_job
        self.assertTrue(assigned_job is not None)
        self.assertEqual(assigned_job.robot_op.sample_index, 1)
        self.assertEqual(assigned_job.robot_op.target_location, end_loc)
        self.assertEqual(assigned_job.station_obj_id, stn_obj_id)
        with self.assertRaises(RobotAssignedRackError):
            t_robot.assign_job(stn_robot_job)

        # complete robot job
        self.assertFalse(t_robot.has_complete_job())
        stn_robot_job.robot_op.output.has_result = True
        stn_robot_job.robot_op.output.success = True
        stn_robot_job.robot_op.output.add_timestamp()
        t_robot.complete_assigned_job(stn_robot_job)
        self.assertTrue(t_robot.assigned_job is None)
        self.assertEqual(t_robot.state, RobotState.EXECUTION_COMPLETE)
        self.assertTrue(t_robot.has_complete_job())

        # retrieve robot job
        ret_job = t_robot.get_complete_job()
        self.assertEqual(t_robot.state, RobotState.IDLE)
        self.assertFalse(t_robot.has_complete_job())
        
        self.assertEqual(ret_job.robot_op.sample_index, 1)
        self.assertEqual(ret_job.robot_op.target_location, end_loc)
        self.assertEqual(ret_job.station_obj_id, stn_obj_id)
        self.assertTrue(ret_job.robot_op.output.has_result)
        self.assertTrue(ret_job.robot_op.output.success)

        history = t_robot.robot_job_history
        self.assertEqual(len(history), 1)
        self.assertEqual(history[0].robot_op.sample_index, 1)
        self.assertEqual(history[0].robot_op.target_location, end_loc)
        self.assertEqual(history[0].station_obj_id, stn_obj_id)
        self.assertTrue(history[0].robot_op.output.has_result)
        self.assertTrue(history[0].robot_op.output.success)

if __name__ == '__main__':
    unittest.main()

        
        
