from bson.objectid import ObjectId
import unittest
from archemist.state.robot import RobotState,RobotTaskType
from archemist.state.robots.kukaLBRIIWA import KukaLBRIIWA, KukaLBRTask
from archemist.util.location import Location
from archemist.util.station_robot_job import StationRobotJob
from archemist.exceptions.exception import RobotAssignedRackError
from mongoengine import connect


class RobotTest(unittest.TestCase):

    def test_robot(self):
        robot_dict = {
            'class': 'KukaLBRIIWA',
            'id': 187,
            'batch_capacity':2
        }

        t_robot = KukaLBRIIWA.from_dict(robot_dict)
        self.assertEqual(t_robot.id, 187)
        self.assertEqual(t_robot.location, Location(node_id=-1, graph_id=-1, frame_name=''))
        t_robot.location = Location(node_id=1, graph_id=7, frame_name='')
        self.assertEqual(t_robot.location, Location(node_id=1, graph_id=7, frame_name=''))
        self.assertEqual(t_robot.state, RobotState.IDLE)
        self.assertEqual(t_robot.operational, True)
        self.assertEqual(t_robot.batch_capacity, 2)

        # Robot job
        # assign job
        self.assertEqual(t_robot.robot_job_history, [])
        stn_obj_id  = ObjectId('0123456789ab0123456789ab')
        robot_job = KukaLBRTask.from_args(name='test_job', parametrs=['False','1'],related_batch_id=32,
                    type=RobotTaskType.LOAD_TO_ROBOT, location=Location(node_id=1, graph_id=7), 
                    origin_station=stn_obj_id)
        t_robot.assign_job(robot_job)
        self.assertEqual(t_robot.state, RobotState.JOB_ASSIGNED)
        assigned_job = t_robot.assigned_job
        self.assertTrue(assigned_job is not None)
        self.assertEqual(assigned_job.name, robot_job.name)
        self.assertEqual(assigned_job.location, robot_job.location)
        self.assertEqual(assigned_job.origin_station, robot_job.origin_station)
        with self.assertRaises(RobotAssignedRackError):
            t_robot.assign_job(robot_job)
        # start executing
        t_robot.start_job_execution()
        self.assertEqual(t_robot.state, RobotState.EXECUTING_JOB)
        # complete robot job
        self.assertFalse(t_robot.has_complete_job())
        t_robot.complete_assigned_job(True)
        self.assertTrue(t_robot.assigned_job is None)
        self.assertEqual(t_robot.state, RobotState.EXECUTION_COMPLETE)
        self.assertTrue(t_robot.has_complete_job())

        # retrieve robot job
        ret_job = t_robot.get_complete_job()
        self.assertEqual(t_robot.state, RobotState.IDLE)
        self.assertFalse(t_robot.has_complete_job())
        self.assertEqual(len(t_robot.onboard_batches),1)
        self.assertEqual(t_robot.onboard_batches[0],32)
        self.assertEqual(ret_job.name, robot_job.name)
        self.assertEqual(ret_job.params, robot_job.params)
        self.assertEqual(ret_job.origin_station, robot_job.origin_station)
        self.assertTrue(ret_job.has_result)
        self.assertTrue(ret_job.was_successful)

        history = t_robot.robot_job_history
        self.assertEqual(len(history), 1)
        self.assertEqual(history[0].name, robot_job.name)
        self.assertEqual(history[0].location, robot_job.location)
        self.assertEqual(history[0].origin_station, robot_job.origin_station)
        self.assertTrue(history[0].has_result)
        self.assertTrue(history[0].was_successful)

if __name__ == '__main__':
    connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    unittest.main()

        
        
