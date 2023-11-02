import unittest

from mongoengine import connect

from archemist.core.state.station import StationState
from archemist.core.state.lot import Lot
from archemist.core.state.batch import Batch
from archemist.stations.lightbox_station.state import (LightBoxStation,
                                                       LBSampleAnalyseRGBOp,
                                                       LBAnalyseRGBResult,
                                                       LBSampleAnalyseLABOp,
                                                       LBAnalyseLABResult)
from archemist.core.state.robot_op import (RobotTaskOpDescriptor,
                                           RobotWaitOpDescriptor)
from archemist.stations.lightbox_station.process import LightBoxProcess
from archemist.core.util.enums import OpOutcome, ProcessStatus
from .testing_utils import test_req_robot_ops, test_req_station_op

class LightBoxStationTest(unittest.TestCase):

    def setUp(self) -> None:
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        station_doc = {
            'type': 'LightBoxStation',
            'id': 23,
            'location': {'coordinates': [1,7], 'descriptor': "LightBoxStation"},
            'total_lot_capacity': 1,
            'handler': 'SimStationOpHandler',
            'properties': {
                "rgb_target_index": 125,
                "lab_target_index": 0.5
            }
        }
        self.station = LightBoxStation.from_dict(station_doc)

    def  tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_state(self):
        # test station is constructed properly
        self.assertIsNotNone(self.station)
        self.assertEqual(self.station.state, StationState.INACTIVE)

        # test station specific members
        self.assertEqual(self.station.rgb_target_index, 125)
        self.assertEqual(self.station.lab_target_index, 0.5)

        # construct lot and add it to station
        batch_1 = Batch.from_args(2)
        lot = Lot.from_args([batch_1])
        self.station.add_lot(lot)


        # test SampleColorOpDescriptor
        t_op = LBSampleAnalyseRGBOp.from_args(lot.batches[0].samples[0])
        self.assertIsNotNone(t_op.object_id)

        # test LBAnalyseRGBResult
        op_result = LBAnalyseRGBResult.from_args(origin_op=t_op.object_id,
                                                 r_value=128,
                                                 g_value=100,
                                                 b_value=12,
                                                 color_index=132,
                                                 target_index=125,
                                                 result_filename="test123.png")
        self.assertEqual(op_result.red_intensity, 128)
        self.assertEqual(op_result.green_intensity, 100)
        self.assertEqual(op_result.blue_intensity, 12)
        self.assertEqual(op_result.color_index, 132)
        self.assertEqual(op_result.color_diff, 7)
        self.assertEqual(op_result.result_filename, "test123.png")

        # test LBSampleAnalyseLABOp
        t_op = LBSampleAnalyseLABOp.from_args(lot.batches[0].samples[0])
        self.assertIsNotNone(t_op.object_id)

        # test LBAnalyseRGBResult
        op_result = LBAnalyseLABResult.from_args(origin_op=t_op.object_id,
                                                 l_value=128,
                                                 a_value=100,
                                                 b_value=12,
                                                 color_index=5.0,
                                                 target_index=3.0,
                                                 result_filename="test123.png")
        self.assertEqual(op_result.l_star_value, 128)
        self.assertEqual(op_result.a_star_value, 100)
        self.assertEqual(op_result.b_star_value, 12)
        self.assertEqual(op_result.color_index, 5.0)
        self.assertEqual(op_result.color_diff, 4.0)
        self.assertEqual(op_result.result_filename, "test123.png")
    
    def test_lightbox_process(self):

        num_samples = 3
        batch_1 = Batch.from_args(num_samples)
        batch_2 = Batch.from_args(num_samples)
        lot = Lot.from_args([batch_1, batch_2])
        
        # add batches to station
        self.station.add_lot(lot)

        # create station process
        operations = [
                {
                    "name": "analyse",
                    "op": "LBSampleAnalyseLABOp",
                    "parameters": None
                }
            ]
        process = LightBoxProcess.from_args(lot=lot,
                                            operations=operations)
        process.lot_slot = 0

        # assert initial state
        self.assertEqual(process.m_state, 'init_state')
        self.assertEqual(process.status, ProcessStatus.INACTIVE)

        # prep_state
        process.tick()
        self.assertEqual(process.m_state, 'prep_state')
        self.assertEqual(process.data['batch_index'], 0)
        self.assertEqual(process.data['sample_index'], 0)

        for i in range(lot.num_batches):
            for j in range(num_samples):
                # load_sample
                process.tick()
                self.assertEqual(process.m_state, 'load_sample')
                test_req_robot_ops(self, process, [RobotTaskOpDescriptor, RobotWaitOpDescriptor])

                # station_process
                process.tick()
                self.assertEqual(process.m_state, 'station_process')
                test_req_station_op(self, process, LBSampleAnalyseLABOp)

                # unload_sample
                process.tick()
                self.assertEqual(process.m_state, 'unload_sample')
                test_req_robot_ops(self, process, [RobotTaskOpDescriptor, RobotWaitOpDescriptor])

                # update_sample_index
                process.tick()
                self.assertEqual(process.m_state, 'update_sample_index')
                self.assertEqual(process.data['batch_index'], i)
                self.assertEqual(process.data['sample_index'], j+1)
            
            # update_batch_index
            process.tick()
            self.assertEqual(process.m_state, 'update_batch_index')
            self.assertEqual(process.data['batch_index'], i+1)
            self.assertEqual(process.data['sample_index'], 0)

        # final_state
        process.tick()
        self.assertEqual(process.m_state, 'final_state')
        self.assertEqual(process.status, ProcessStatus.FINISHED)

if __name__ == '__main__':
    mongoengine.connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    unittest.main()