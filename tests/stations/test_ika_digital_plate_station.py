import unittest
from time import sleep
from mongoengine import connect
from bson.objectid import ObjectId

from archemist.stations.ika_digital_plate_station.state import (IKADigitalPlateStation, IKADigitalPlateMode,
                                                                IKAHeatStirBatchOp,
                                                                IKAStirBatchOp,
                                                                IKAHeatBatchOp,
                                                                IKAStopOp)
from archemist.stations.ika_digital_plate_station.process import PXRDWorkflowStirringProcess, PandaIKASolubilityProcess
from archemist.stations.ika_digital_plate_station.handler import SimIKAPlateDigitalHandler
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.util.enums import StationState, OpOutcome, ProcessStatus
from archemist.core.state.robot_op import DropBatchOp, RobotTaskOp
from .testing_utils import test_req_station_op, test_req_robot_ops, test_req_station_proc


class IKADigitalPlateStationTest(unittest.TestCase):
    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        self.station_doc = {
            'type': 'IKADigitalPlateStation',
            'id': 20,
            'location': {'coordinates': [1, 7], 'descriptor': "IKADigitalPlateStation"},
            'total_lot_capacity': 1,
            'handler': 'SimStationOpHandler',
            'properties': None,
            'materials': None
        }

        self.station = IKADigitalPlateStation.from_dict(self.station_doc)

    def tearDown(self):
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_state(self):
        # test station is constructed properly
        self.assertIsNotNone(self.station)
        self.assertEqual(self.station.state, StationState.INACTIVE)
        self.assertIsNone(self.station.mode)

        self.assertIsNone(self.station.current_temperature)
        self.station.current_temperature = 20
        self.assertEqual(self.station.current_temperature, 20)

        self.assertIsNone(self.station.current_stirring_speed)
        self.station.current_stirring_speed = 120
        self.assertEqual(self.station.current_stirring_speed, 120)

        self.assertIsNone(self.station.external_temperature)
        self.station.external_temperature = 25
        self.assertEqual(self.station.external_temperature, 25)

        self.assertIsNone(self.station.viscosity_trend)
        self.station.viscosity_trend = 0.8
        self.assertEqual(self.station.viscosity_trend, 0.8)

        # construct lot and add it to station
        batch_1 = Batch.from_args(2)
        lot = Lot.from_args([batch_1])
        self.station.add_lot(lot)

        # test IKAStirBatchOp
        t_op = IKAStirBatchOp.from_args(target_batch=batch_1,
                                        target_stirring_speed=500,
                                        duration=10,
                                        time_unit="minute")

        self.assertEqual(t_op.target_stirring_speed, 500)
        self.assertEqual(t_op.duration, 10)
        self.assertEqual(t_op.time_unit, "minute")

        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.assertTrue(self.station.mode, IKADigitalPlateMode.STIRRING)
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertIsNone(self.station.mode)

        # test IKAHeatStirBatchOp
        t_op = IKAHeatStirBatchOp.from_args(target_batch=batch_1,
                                            target_temperature=100,
                                            target_stirring_speed=500,
                                            duration=10,
                                            time_unit="minute")

        self.assertEqual(t_op.target_stirring_speed, 500)
        self.assertEqual(t_op.target_temperature, 100)
        self.assertEqual(t_op.duration, 10)
        self.assertEqual(t_op.time_unit, "minute")

        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.assertTrue(self.station.mode, IKADigitalPlateMode.HEATING_STIRRING)
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertIsNone(self.station.mode)

        # test IKAHeatBatchOp
        t_op = IKAHeatBatchOp.from_args(target_batch=batch_1,
                                        target_temperature=100,
                                        duration=10,
                                        time_unit="minute")

        self.assertEqual(t_op.target_temperature, 100)
        self.assertEqual(t_op.duration, 10)
        self.assertEqual(t_op.time_unit, "minute")

        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.assertTrue(self.station.mode, IKADigitalPlateMode.HEATING)
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertIsNone(self.station.mode)

        # test IKAHeatBatchOp with no duration
        t_op = IKAHeatBatchOp.from_args(target_batch=batch_1,
                                        target_temperature=100,
                                        duration=-1,
                                        time_unit=None)

        self.assertEqual(t_op.target_temperature, 100)
        self.assertEqual(t_op.duration, -1)

        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.assertTrue(self.station.mode, IKADigitalPlateMode.HEATING)
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertTrue(self.station.mode, IKADigitalPlateMode.HEATING)

        # test IKAStopOp
        t_op = IKAStopOp.from_args()

        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertIsNone(self.station.mode)

    def test_pxrd_workflow_process(self):
        batch_1 = Batch.from_args(3)
        batch_2 = Batch.from_args(3)
        lot = Lot.from_args([batch_1, batch_2])

        # add batches to station
        self.station.add_lot(lot)

        # create station process
        operations = [
            {
                "name": "stir_op",
                "op": "IKAStirBatchOp",
                "parameters": {
                        "target_stirring_speed": 500,
                        "duration": 5,
                        "time_unit": "minute"
                }
            }
        ]
        process = PXRDWorkflowStirringProcess.from_args(lot=lot,
                                                        eight_well_rack_first=True,
                                                        operations=operations)
        process.lot_slot = 0

        # assert initial state
        self.assertEqual(process.m_state, 'init_state')
        self.assertEqual(process.status, ProcessStatus.INACTIVE)

        # prep_state
        process.tick()
        self.assertEqual(process.m_state, 'prep_state')

        # place_lot
        process.tick()
        self.assertEqual(process.m_state, 'place_lot')
        test_req_robot_ops(self, process, [DropBatchOp]*2)

        # load_stir_plate
        process.tick()
        self.assertEqual(process.m_state, 'load_stir_plate')
        test_req_robot_ops(self, process, [RobotTaskOp])

        # stir
        process.tick()
        self.assertEqual(process.m_state, 'stir')
        test_req_station_op(self, process, IKAStirBatchOp)

        # final_state
        process.tick()
        self.assertEqual(process.m_state, 'final_state')
        self.assertEqual(process.status, ProcessStatus.FINISHED)

    def test_panda_solubility_process(self):
        batch_1 = Batch.from_args(1)
        lot = Lot.from_args([batch_1])

        # add batches to station
        self.station.add_lot(lot)

        # create station process
        operations = [
            {
                "name": "stir_heat_op",
                "op": "IKAHeatStirBatchOp",
                "parameters": {
                        "target_temperature": 100,
                        "target_stirring_speed": 500,
                        "duration": -1,
                        "time_unit": "second"
                }
            },
            {
                "name": "stop_op",
                "op": "IKAStopOp",
                "parameters": None
            }
        ]
        process = PandaIKASolubilityProcess.from_args(lot=lot,
                                                      operations=operations)
        process.lot_slot = 0

        # assert initial state
        self.assertEqual(process.m_state, 'init_state')
        self.assertEqual(process.status, ProcessStatus.INACTIVE)

        # prep_state
        process.tick()
        self.assertEqual(process.m_state, 'prep_state')
        self.assertIsNone(process.data['start_time'])

        # load_ika_plate
        process.tick()
        self.assertEqual(process.m_state, 'load_ika_plate')
        test_req_robot_ops(self, process, [RobotTaskOp])

        # start_stirring_heating
        process.tick()
        self.assertEqual(process.m_state, 'start_stirring_heating')
        test_req_station_op(self, process, IKAHeatStirBatchOp)

        # sleep
        process.tick()
        self.assertEqual(process.m_state, 'sleep')
        sleep(2)

        # check_solubility
        process.tick()
        self.assertEqual(process.m_state, 'check_solubility')
        from archemist.stations.solubility_station.process import PandaCheckSolubilityProcess
        test_req_station_proc(self, process, PandaCheckSolubilityProcess)

        # manually add solubility result to advance state
        from archemist.stations.solubility_station.state import SolubilityOpResult, SolubilityState
        solubility_result = SolubilityOpResult.from_args(origin_op=ObjectId(),
                                                         solubility_state=SolubilityState.UNDISSOLVED,
                                                         result_filename="some_file.png")
        batch_1.samples[0].add_result_op(solubility_result)

        # liquid_addition
        process.tick()
        self.assertEqual(process.m_state, 'liquid_addition')
        from archemist.stations.peristaltic_pumps_station.process import PandaPumpSolubilityProcess
        test_req_station_proc(self, process, PandaPumpSolubilityProcess)

        # sleep
        process.tick()
        self.assertEqual(process.m_state, 'sleep')
        sleep(2)

        # check_solubility
        process.tick()
        self.assertEqual(process.m_state, 'check_solubility')
        test_req_station_proc(self, process, PandaCheckSolubilityProcess)

        # manually add solubility result to advance state
        solubility_result = SolubilityOpResult.from_args(origin_op=ObjectId(),
                                                         solubility_state=SolubilityState.DISSOLVED,
                                                         result_filename="some_file.png")
        batch_1.samples[0].add_result_op(solubility_result)

        # stop_stirring_heating
        process.tick()
        self.assertEqual(process.m_state, 'stop_stirring_heating')
        test_req_station_op(self, process, IKAStopOp)

        # unload_ika_plate
        process.tick()
        self.assertEqual(process.m_state, 'unload_ika_plate')
        test_req_robot_ops(self, process, [RobotTaskOp])

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
        handler = SimIKAPlateDigitalHandler(self.station)

        # initialise the handler
        self.assertTrue(handler.initialise())

        # construct op
        t_op = IKAHeatStirBatchOp.from_args(target_batch=batch_1,
                                            target_temperature=100,
                                            target_stirring_speed=500,
                                            duration=10,
                                            time_unit="minute")
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()

        # get op
        parameters = {}
        parameters["target_temperature"] = t_op.target_temperature
        parameters["target_stirring_speed"] = t_op.target_stirring_speed
        parameters["duration"] = t_op.duration
        parameters["time_unit"] = t_op.time_unit

        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(op_results), 1)
        self.assertEqual(op_results[0].origin_op, t_op.object_id)
        self.assertDictEqual(op_results[0].parameters, parameters)


if __name__ == '__main__':
    unittest.main()
