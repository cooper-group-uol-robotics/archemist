import unittest
from datetime import datetime

from mongoengine import connect

from archemist.stations.chemspeed_flex_station.state import (ChemSpeedFlexStation,
                                                             ChemSpeedJobStatus,
                                                             CSOpenDoorOp,
                                                             CSCloseDoorOp,
                                                             CSLiquidDispenseOp,
                                                             CSRunJobOp)
from archemist.core.state.robot_op import (RobotNavOp,
                                           RobotWaitOp,
                                           CollectBatchOp,
                                           DropBatchOp)
from archemist.core.state.station_op_result import MaterialOpResult
from archemist.stations.chemspeed_flex_station.process import CMFlexLiquidDispenseProcess
from archemist.core.state.lot import Lot
from archemist.core.state.batch import Batch
from archemist.core.util.enums import StationState, OpOutcome, ProcessStatus
from .testing_utils import test_req_robot_ops, test_req_station_op


class ChemspeedFlexStationTest(unittest.TestCase):
    def setUp(self) -> None:
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        station_doc = {
            'type': 'ChemSpeedFlexStation',
            'id': 22,
            'location': {'coordinates': [1, 7], 'descriptor': "ChemSpeedFlexStation"},
            'total_lot_capacity': 1,
            'handler': 'SimStationOpHandler',
            'materials':
            {
                'liquids':
                [{
                    'name': 'water',
                    'amount': 400,
                    'unit': 'mL',
                    'density': 997,
                    'density_unit': "kg/m3",
                    "details": None,
                    'expiry_date': datetime.fromisoformat('2025-02-11')
                }]
            }
        }

        self.station = ChemSpeedFlexStation.from_dict(station_doc)

    def tearDown(self) -> None:
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
        self.assertEqual(self.station.job_status, ChemSpeedJobStatus.INVALID)
        self.assertTrue(self.station.door_closed)
        self.assertEqual(len(self.station.liquids_dict), 1)
        liquid = self.station.liquids_dict['water']
        self.assertIsNotNone(liquid)
        self.assertEqual(liquid.volume, 400)

        # test CSOpenDoorOp
        t_op = CSOpenDoorOp.from_args()
        self.assertIsNotNone(t_op)
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertFalse(self.station.door_closed)

        # test CSCloseDoorOp
        t_op = CSCloseDoorOp.from_args()
        self.assertIsNotNone(t_op)
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertTrue(self.station.door_closed)

        # test CSProcessingOp
        t_op = CSRunJobOp.from_args()
        self.assertIsNotNone(t_op)
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.assertEqual(self.station.job_status, ChemSpeedJobStatus.RUNNING_JOB)
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertEqual(self.station.job_status, ChemSpeedJobStatus.JOB_COMPLETE)

        # test CSProcessingOp
        t_op = CSLiquidDispenseOp.from_args(target_lot=lot,
                                            dispense_table={'water': [10.0, 20.0]},
                                            dispense_unit="mL")
        self.assertIsNotNone(t_op)
        self.assertDictEqual(dict(t_op.dispense_table), {'water': [10.0, 20.0]})
        self.assertEqual(t_op.dispense_unit, "mL")
        self.assertEqual(t_op.to_csv_string(), r"10.0\n20.0\n")

        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.assertEqual(self.station.job_status, ChemSpeedJobStatus.RUNNING_JOB)
        self.assertEqual(liquid.volume, 400)

        # create results for operation
        materials_names = [material_name for material_name in t_op.dispense_table.keys()]
        samples_qtys = zip(*[qtys for qtys in t_op.dispense_table.values()])
        dispense_results = []
        for qty_tuple in samples_qtys:
            sample_op_result = MaterialOpResult.from_args(origin_op=t_op.object_id,
                                                          material_names=materials_names,
                                                          amounts=list(qty_tuple),
                                                          units=[t_op.dispense_unit]*len(materials_names))
            dispense_results.append(sample_op_result)

        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, dispense_results)
        self.assertEqual(self.station.job_status, ChemSpeedJobStatus.JOB_COMPLETE)
        self.assertEqual(liquid.volume, 370.0)

    def test_chemspeed_process(self):

        batch_1 = Batch.from_args(2)
        batch_2 = Batch.from_args(2)
        lot = Lot.from_args([batch_1, batch_2])

        # add batches to station
        self.station.add_lot(lot)

        # create station process
        operations = [
            {
                "name": "dispense_op",
                "op": "CSLiquidDispenseOp",
                "parameters": {
                        "dispense_table": {"water": [10.0, 20.0]},
                        "dispense_unit": "mL"
                }
            }
        ]
        process = CMFlexLiquidDispenseProcess.from_args(lot=lot,
                                                        operations=operations)
        process.lot_slot = 0

        # assert initial state
        self.assertEqual(process.m_state, 'init_state')
        self.assertEqual(process.status, ProcessStatus.INACTIVE)

        # prep_state
        process.tick()
        self.assertEqual(process.status, ProcessStatus.RUNNING)
        self.assertEqual(process.m_state, 'prep_state')
        self.assertFalse(process.data['dispense_finished'])

        # navigate_to_chemspeed and wait
        process.tick()
        self.assertEqual(process.m_state, 'navigate_to_chemspeed')
        test_req_robot_ops(self, process, [RobotNavOp, RobotWaitOp])

        # open_chemspeed_door
        process.tick()
        self.assertEqual(process.m_state, 'open_chemspeed_door')
        test_req_station_op(self, process, CSOpenDoorOp)

        # load_lot
        process.tick()
        self.assertEqual(process.m_state, 'load_lot')
        test_req_robot_ops(self, process, [DropBatchOp]*2)

        # close_chemspeed_door
        process.tick()
        self.assertEqual(process.m_state, 'close_chemspeed_door')
        test_req_station_op(self, process, CSCloseDoorOp)

        # retreat_from_chemspeed
        process.tick()
        self.assertEqual(process.m_state, 'retreat_from_chemspeed')
        test_req_robot_ops(self, process, [RobotNavOp])

        # chemspeed_process
        process.tick()
        self.assertEqual(process.m_state, 'chemspeed_process')
        test_req_station_op(self, process, CSLiquidDispenseOp)

        # navigate_to_chemspeed
        process.tick()
        self.assertEqual(process.m_state, 'navigate_to_chemspeed')
        test_req_robot_ops(self, process, [RobotNavOp, RobotWaitOp])

        # open_chemspeed_door
        process.tick()
        self.assertEqual(process.m_state, 'open_chemspeed_door')
        test_req_station_op(self, process, CSOpenDoorOp)

        # unload_lot
        process.tick()
        self.assertEqual(process.m_state, 'unload_lot')
        test_req_robot_ops(self, process, [CollectBatchOp]*2)

        # close_chemspeed_door
        process.tick()
        self.assertEqual(process.m_state, 'close_chemspeed_door')
        test_req_station_op(self, process, CSCloseDoorOp)

        # retreat_from_chemspeed
        process.tick()
        self.assertEqual(process.m_state, 'retreat_from_chemspeed')
        test_req_robot_ops(self, process, [RobotNavOp])

        # final_state
        process.tick()
        self.assertEqual(process.m_state, 'final_state')
        self.assertEqual(process.status, ProcessStatus.FINISHED)


if __name__ == '__main__':
    unittest.main()
