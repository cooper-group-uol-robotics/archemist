import unittest
from mongoengine import connect
from transitions import State


from archemist.core.state.robot_op import RobotOpDescriptor
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.state.station_process import StationProcess, ProcessStatus
from archemist.core.state.lot import Lot, Batch
from archemist.core.util.location import Location

class TestProcess(StationProcess):
    STATES = [State(name='init_state'),
        State(name='prep_state', on_enter='initialise_process_data'), 
        State(name='pickup_batch', on_enter=['request_pickup_batch']),
        State(name='final_state')]
    
    TRANSITIONS = [
        {'source':'init_state','dest':'prep_state'},
        {'source':'prep_state','dest':'pickup_batch'},
        {'source':'pickup_batch','dest':'final_state', 'conditions':'are_req_robot_ops_completed'}
    ]

    def initialise_process_data(self):
        self._model_proxy.data['batch_index'] = 0

    def request_pickup_batch(self):
        robot_op = RobotOpDescriptor.construct_op()
        self.request_robot_op(robot_op)

class StationProcessTest(unittest.TestCase):
    
    def setUp(self):
        connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')

    def test_station_process_fields(self):
        # construct lot
        batch_1 = Batch.from_arguments(3, Location(1, 2, "some_frame"))
        batch_2 = Batch.from_arguments(3, Location(1, 2, "some_frame"))
        lot = Lot.from_args([batch_1, batch_2])
        # construct process
        proc = StationProcess.from_args(lot, 1)
        self.assertIsNotNone(proc.uuid)
        self.assertEqual(proc.status, ProcessStatus.INACTIVE)
        self.assertEqual(proc.m_state, "init_state")
        self.assertEqual(proc.processing_slot, 1)
        proc.processing_slot = 3
        self.assertEqual(proc.processing_slot, 3)
        
        # test data field
        self.assertEqual(len(proc.data), 0)
        proc.data["rack_index"] = 3
        self.assertEqual(len(proc.data), 1)
        self.assertEqual(proc.data["rack_index"], 3)

        # test lot
        proc_lot  = proc.lot
        self.assertEqual(proc_lot.uuid, lot.uuid)
        self.assertEqual(proc_lot.num_batches, 2)
        self.assertEqual(proc_lot.batches[0].uuid, batch_1.uuid)

        # test robot_op fields
        self.assertEqual(len(proc.req_robot_ops), 0)
        self.assertEqual(len(proc.robot_ops_history), 0)
        
        robot_op = RobotOpDescriptor.construct_op()
        proc.request_robot_op(robot_op)
        self.assertFalse(proc.are_req_robot_ops_completed())
        self.assertEqual(len(proc.req_robot_ops), 1)
        self.assertEqual(proc.status, ProcessStatus.WAITING_ON_ROBOT_OPS)

        proc.complete_robot_op(robot_op)
        self.assertEqual(len(proc.req_robot_ops), 0)
        self.assertTrue(proc.are_req_robot_ops_completed())
        self.assertEqual(len(proc.robot_ops_history), 1)
        self.assertEqual(proc.status, ProcessStatus.RUNNING)

        # test station_op fields
        self.assertEqual(len(proc.req_station_ops), 0)
        self.assertEqual(len(proc.station_ops_history), 0)
        
        station_op = StationOpDescriptor.construct_op()
        proc.request_station_op(station_op)
        self.assertFalse(proc.are_req_station_ops_completed())
        self.assertEqual(len(proc.req_station_ops), 1)
        self.assertEqual(proc.status, ProcessStatus.WAITING_ON_STATION_OPS)

        proc.complete_station_op(station_op)
        self.assertEqual(len(proc.req_station_ops), 0)
        self.assertTrue(proc.are_req_station_ops_completed())
        self.assertEqual(len(proc.station_ops_history), 1)
        self.assertEqual(proc.status, ProcessStatus.RUNNING)

        # test station_proc fields
        self.assertEqual(len(proc.req_station_procs), 0)
        self.assertEqual(len(proc.station_procs_history), 0)
        
        station_proc = StationProcess.from_args(lot, 2)
        proc.request_station_process(station_proc)
        self.assertFalse(proc.are_req_station_procs_completed())
        self.assertEqual(len(proc.req_station_procs), 1)
        self.assertEqual(proc.status, ProcessStatus.WAITING_ON_STATION_PROCS)

        proc.complete_station_proc(station_proc)
        self.assertEqual(len(proc.req_station_procs), 0)
        self.assertTrue(proc.are_req_station_procs_completed())
        self.assertEqual(len(proc.station_procs_history), 1)
        self.assertEqual(proc.status, ProcessStatus.RUNNING)

    def test_station_process_state_machine(self):
        # construct lot
        batch_1 = Batch.from_arguments(3, Location(1, 2, "some_frame"))
        batch_2 = Batch.from_arguments(3, Location(1, 2, "some_frame"))
        lot = Lot.from_args([batch_1, batch_2])
        # construct process
        proc = TestProcess.from_args(lot, 1)
        self.assertEqual(proc.status, ProcessStatus.INACTIVE)
        self.assertIsNone(proc._state_machine)
        self.assertEqual(proc.m_state, "init_state")

        # tick process
        # transition to prep_state
        proc.tick()
        self.assertEqual(proc.status, ProcessStatus.RUNNING)
        self.assertIsNotNone(proc._state_machine)
        self.assertEqual(proc.m_state, "prep_state")
        self.assertEqual(proc.data["batch_index"], 0)
        self.assertEqual(len(proc.req_robot_ops), 0)

        # transition to pickup_batch
        proc.tick()
        self.assertEqual(proc.status, ProcessStatus.WAITING_ON_ROBOT_OPS)
        self.assertEqual(proc.m_state, "pickup_batch")
        self.assertEqual(len(proc.req_robot_ops), 1)

        # no transition
        proc.tick()
        self.assertEqual(proc.status, ProcessStatus.WAITING_ON_ROBOT_OPS)
        self.assertEqual(proc.m_state, "pickup_batch")
        self.assertEqual(len(proc.req_robot_ops), 1)

        # transition to final_state
        robot_op = proc.req_robot_ops[0]
        proc.complete_robot_op(robot_op)
        proc.tick()
        self.assertEqual(proc.status, ProcessStatus.FINISHED)
        self.assertEqual(len(proc.robot_ops_history), 1)
        self.assertEqual(proc.m_state, "final_state")

if __name__ == "__main__":
    unittest.main()