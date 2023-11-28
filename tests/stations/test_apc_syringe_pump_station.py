import unittest
from datetime import datetime

from mongoengine import connect

from archemist.stations.apc_syringe_pump_station.state import (APCSyringePumpStation,
                                                              APCSPumpDispenseVolumeOp,
                                                              APCSPumpDispenseRateOp,
                                                              APCSPumpFinishDispensingOp)
from archemist.stations.apc_syringe_pump_station.handler import SimAPCSyringePumpStationHandler
from archemist.core.util.enums import StationState, OpOutcome
from archemist.core.state.lot import Lot, Batch
from archemist.core.state.station_op_result import MaterialOpResult, ProcessOpResult
from .testing_utils import test_req_robot_ops, test_req_station_op

class PeristalticLiquidDispensingTest(unittest.TestCase):
    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        self.station_dict = {
            'type': 'APCSyringePumpStation',
            'id': 20,
            'location': {'coordinates': [1,7], 'descriptor': "APCSyringePumpStation"},
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
                    "details": {"in_port": 3, "out_port": 4},
                    'expiry_date': datetime.fromisoformat('2025-02-11')
                }]
            },
            'properties': None
        }
                
    def tearDown(self):
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()
    
    def test_state(self):
        # test station is constructed properly
        station = APCSyringePumpStation.from_dict(self.station_dict)
        self.assertIsNotNone(station)
        self.assertEqual(station.state, StationState.INACTIVE)

        # construct lot and add it to station
        batch_1 = Batch.from_args(2)
        lot = Lot.from_args([batch_1])
        station.add_lot(lot)

        # test APCSPumpDispenseVolumeOp
        t_op = APCSPumpDispenseVolumeOp.from_args(target_sample=lot.batches[0].samples[0],
                                            liquid_name='water',
                                            dispense_volume=100,
                                            dispense_unit="mL",
                                            dispense_rate=3.5,
                                            rate_unit="mL/minute")
        self.assertIsNotNone(t_op.object_id)
        self.assertEqual(t_op.liquid_name, "water")
        self.assertEqual(t_op.dispense_volume, 100)
        self.assertEqual(t_op.dispense_unit, "mL")
        self.assertEqual(t_op.dispense_rate, 3.5)
        self.assertEqual(t_op.rate_unit, "mL/minute")

        station.add_station_op(t_op)
        station.update_assigned_op()
        op_result = MaterialOpResult.from_args(t_op.object_id,
                                               [t_op.liquid_name],
                                               [t_op.dispense_volume],
                                               [t_op.dispense_unit])

        station.complete_assigned_op(OpOutcome.SUCCEEDED, [op_result])
        self.assertEqual(station.liquids_dict["water"].volume, 300)
        self.assertEqual(station.liquids_dict["water"].volume_unit, "mL")

        # test APCSPumpDispenseRateOp
        t_op = APCSPumpDispenseRateOp.from_args(target_sample=lot.batches[0].samples[0],
                                                liquid_name='water',
                                                dispense_rate=5.0,
                                                rate_unit="mL/minute")
        self.assertIsNotNone(t_op.object_id)
        self.assertEqual(t_op.liquid_name, "water")
        self.assertEqual(t_op.dispense_rate, 5.0)
        self.assertEqual(t_op.rate_unit, "mL/minute")

        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)


        # test APCSPumpFinishDispensingOp
        t_op = APCSPumpFinishDispensingOp.from_args(target_sample=lot.batches[0].samples[0],
                                            liquid_name='water')
        self.assertIsNotNone(t_op.object_id)
        self.assertEqual(t_op.liquid_name, "water")

        station.add_station_op(t_op)
        station.update_assigned_op()
        op_result = MaterialOpResult.from_args(t_op.object_id,
                                               [t_op.liquid_name],
                                               [50],
                                               ["mL"])

        station.complete_assigned_op(OpOutcome.SUCCEEDED, [op_result])
        self.assertEqual(station.liquids_dict["water"].volume, 250)
        self.assertEqual(station.liquids_dict["water"].volume_unit, "mL")

    def test_sim_handler(self):
        station = APCSyringePumpStation.from_dict(self.station_dict)
        batch_1 = Batch.from_args(3)
        lot = Lot.from_args([batch_1])

        # add batches to station
        station.add_lot(lot)

        # construct handler
        handler = SimAPCSyringePumpStationHandler(station)

        # initialise the handler
        self.assertTrue(handler.initialise())

        # test handling APCSPumpDispenseVolumeOp
        t_op = APCSPumpDispenseVolumeOp.from_args(target_sample=lot.batches[0].samples[0],
                                            liquid_name='water',
                                            dispense_volume=100,
                                            dispense_unit="mL",
                                            dispense_rate=3.5,
                                            rate_unit="mL/minute")
        station.add_station_op(t_op)
        station.update_assigned_op()
        
        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(op_results), 1)
        self.assertEqual(op_results[0].origin_op, t_op.object_id)
        self.assertTrue(isinstance(op_results[0], MaterialOpResult))
        self.assertEqual(op_results[0].material_names[0], "water")
        self.assertEqual(op_results[0].amounts[0], 100)
        self.assertEqual(op_results[0].units[0], "mL")
        station.complete_assigned_op(outcome, op_results)
        self.assertEqual(station.liquids_dict["water"].volume, 300)

        # test handling APCSPumpDispenseRateOp
        t_op = APCSPumpDispenseRateOp.from_args(target_sample=lot.batches[0].samples[0],
                                            liquid_name='water',
                                            dispense_rate=3.5,
                                            rate_unit="mL/minute")
        station.add_station_op(t_op)
        station.update_assigned_op()
        
        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(op_results), 1)
        self.assertEqual(op_results[0].origin_op, t_op.object_id)
        self.assertTrue(isinstance(op_results[0], ProcessOpResult))
        self.assertEqual(op_results[0].parameters["dispense_rate"], 3.5)
        self.assertEqual(op_results[0].parameters["rate_unit"], "mL/minute")
        station.complete_assigned_op(outcome, op_results)

        # test handling APCSPumpFinishDispensingOp
        t_op = APCSPumpFinishDispensingOp.from_args(target_sample=lot.batches[0].samples[0],
                                            liquid_name='water')
        station.add_station_op(t_op)
        station.update_assigned_op()
        
        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(op_results), 1)
        self.assertEqual(op_results[0].origin_op, t_op.object_id)
        self.assertTrue(isinstance(op_results[0], MaterialOpResult))
        self.assertEqual(op_results[0].material_names[0], "water")
        self.assertGreater(op_results[0].amounts[0], 0)
        self.assertEqual(op_results[0].units[0], "mL")
        station.complete_assigned_op(outcome, op_results)
        self.assertLessEqual(station.liquids_dict["water"].volume, 300)
