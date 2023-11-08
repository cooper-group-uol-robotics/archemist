import unittest
from typing import List
from datetime import datetime

from mongoengine import connect
from archemist.core.state.robot_op import (RobotTaskOp,
                                           RobotWaitOp,
                                           CollectBatchOp,
                                           DropBatchOp)
from archemist.stations.pxrd_station.state import (PXRDStation,
                                                   PXRDJobStatus,
                                                   PXRDAnalysisOp, 
                                                   PXRDCloseDoorOp,
                                                   PXRDOpenDoorOp,
                                                   PXRDAnalysisResult)
from archemist.stations.pxrd_station.process import PXRDWorkflowAnalysisProcess
from archemist.stations.pxrd_station.handler import SimPXRDStationHandler
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot

from archemist.core.util.enums import StationState, OpOutcome, ProcessStatus
from .testing_utils import test_req_robot_ops, test_req_station_op

class PXRDStationTest(unittest.TestCase):
    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        self.station_doc = {
            'type': 'PXRDStation',
            'id': 22,
            'location': {'coordinates': [1,7], 'descriptor': "ChemSpeedFlexStation"},
            'total_lot_capacity': 1,
            'handler': 'SimStationOpHandler',
            'materials': None,
            'parameters': None
        }

        self.station = PXRDStation.from_dict(self.station_doc)

    def tearDown(self):
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_state(self):
        # test station is constructed properly
        self.assertIsNotNone(self.station)
        self.assertEqual(self.station.state, StationState.INACTIVE)

        # construct lot and add it to station
        batch_1 = Batch.from_args(2)
        lot = Lot.from_args([batch_1])
        self.station.add_lot(lot)

        # test station specific members
        self.assertEqual(self.station.job_status, PXRDJobStatus.INVALID)
        self.assertTrue(self.station.door_closed)
        
        # test PXRDOpenDoorOp
        t_op = PXRDOpenDoorOp.from_args()
        self.assertIsNotNone(t_op)
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertFalse(self.station.door_closed)

        # test PXRDOpenDoorOp
        t_op = PXRDCloseDoorOp.from_args()
        self.assertIsNotNone(t_op)
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertTrue(self.station.door_closed)

        # test PXRDAnalysisOp
        t_op = PXRDAnalysisOp.from_args(target_batch=batch_1)
        self.assertIsNotNone(t_op)
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.assertEqual(self.station.job_status, PXRDJobStatus.RUNNING_JOB)
        
        # test PXRDAnalysisResult
        t_result = PXRDAnalysisResult.from_args(t_op.object_id, "some_file.xml")
        self.assertIsNotNone(t_result)
        self.assertEqual(t_result.result_filename, "some_file.xml")

        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, [t_result])
        self.assertEqual(self.station.job_status, PXRDJobStatus.JOB_COMPLETE)

    def test_pxrd_process(self):
        # construct batches
        batch_1 = Batch.from_args(2)
        batch_2 = Batch.from_args(2)
        lot = Lot.from_args([batch_1, batch_2])
        
        # add lot to station
        self.station.add_lot(lot)

        # create station process
        operations = [
                {
                    "name": "analyse_op",
                    "op": "PXRDAnalysisOp",
                    "parameters": None
                }
            ]
        process = PXRDWorkflowAnalysisProcess.from_args(lot=lot,
                                                        eight_well_rack_first=True,
                                                        operations=operations)
        process.lot_slot = 0

        # assert initial state
        self.assertEqual(process.m_state, 'init_state')
        self.assertEqual(process.status, ProcessStatus.INACTIVE)

        # prep_state
        process.tick()
        self.assertEqual(process.status, ProcessStatus.RUNNING)
        self.assertEqual(process.m_state, 'prep_state')
        self.assertFalse(process.data['batch_analysed'])

        # open_pxrd_door
        process.tick()
        self.assertEqual(process.m_state, 'open_pxrd_door')
        test_req_robot_ops(self, process, [RobotTaskOp, RobotWaitOp])
        
        # open_pxrd_door_update
        process.tick()
        self.assertEqual(process.m_state, 'open_pxrd_door_update')
        test_req_station_op(self, process, PXRDOpenDoorOp)

        # load_pxrd
        process.tick()
        self.assertEqual(process.m_state, 'load_pxrd')
        test_req_robot_ops(self, process, [DropBatchOp, RobotWaitOp])

        # close_pxrd_door
        process.tick()
        self.assertEqual(process.m_state, 'close_pxrd_door')
        test_req_robot_ops(self, process, [RobotTaskOp, RobotWaitOp])
        
        # close_pxrd_door_update
        process.tick()
        self.assertEqual(process.m_state, 'close_pxrd_door_update')
        test_req_station_op(self, process, PXRDCloseDoorOp)

        # pxrd_process
        process.tick()
        self.assertEqual(process.m_state, 'pxrd_process')
        test_req_station_op(self, process, PXRDAnalysisOp)

        # open_pxrd_door
        process.tick()
        self.assertEqual(process.m_state, 'open_pxrd_door')
        test_req_robot_ops(self, process, [RobotTaskOp, RobotWaitOp])
        
        # open_pxrd_door_update
        process.tick()
        self.assertEqual(process.m_state, 'open_pxrd_door_update')
        test_req_station_op(self, process, PXRDOpenDoorOp)

        # unload_pxrd
        process.tick()
        self.assertEqual(process.m_state, 'unload_pxrd')
        test_req_robot_ops(self, process, [CollectBatchOp, RobotWaitOp])

        # close_pxrd_door
        process.tick()
        self.assertEqual(process.m_state, 'close_pxrd_door')
        test_req_robot_ops(self, process, [RobotTaskOp, RobotWaitOp])
        
        # close_pxrd_door_update
        process.tick()
        self.assertEqual(process.m_state, 'close_pxrd_door_update')
        test_req_station_op(self, process, PXRDCloseDoorOp)

        # final_state
        process.tick()
        self.assertEqual(process.m_state, 'final_state')
        self.assertEqual(process.status, ProcessStatus.FINISHED)

    def test_sim_handler(self):
        batch_1 = Batch.from_args(3)
        lot = Lot.from_args([batch_1])

        # add batches to station
        self.station.add_lot(lot)

        # construct handler
        handler = SimPXRDStationHandler(self.station)

        # initialise the handler
        self.assertTrue(handler.initialise())

        # construct analyse op
        t_op = PXRDAnalysisOp.from_args(target_batch=batch_1)
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        
        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(op_results), 1)
        self.assertEqual(op_results[0].origin_op, t_op.object_id)
        self.assertTrue(isinstance(op_results[0], PXRDAnalysisResult))
        self.assertIsNotNone(op_results[0].result_filename)
        self.station.complete_assigned_op(outcome, op_results)

        # construct analyse op
        t_op = PXRDOpenDoorOp.from_args()
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        
        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertIsNone(op_results)

if __name__ == '__main__':
    unittest.main()
