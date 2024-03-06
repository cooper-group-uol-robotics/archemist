import unittest
from datetime import datetime

from mongoengine import connect

from archemist.stations.diaphragm_pump_station.state import (DiaphragmPumpStation,
                                                             DiaphragmPumpDispenseVolumeOp)
from archemist.stations.diaphragm_pump_station.handler import SimDiaphragmPumpStationHandler
from archemist.core.util.enums import StationState, OpOutcome
from archemist.core.state.lot import Lot, Batch
from archemist.core.state.station_op_result import MaterialOpResult


class DiaphragmPumpStationTest(unittest.TestCase):
    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        self.station_dict = {
            'type': 'DiaphragmPumpStation',
            'id': 20,
            'location': {'coordinates': [1, 7], 'descriptor': "APCSyringePumpStation"},
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
            },
            'properties': None
        }

    def tearDown(self):
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_state(self):
        # test station is constructed properly
        station = DiaphragmPumpStation.from_dict(self.station_dict)
        self.assertIsNotNone(station)
        self.assertEqual(station.state, StationState.INACTIVE)

        # construct lot and add it to station
        batch_1 = Batch.from_args(2)
        lot = Lot.from_args([batch_1])
        station.add_lot(lot)

        # test DiaphragmPumpDispenseVolumeOp
        t_op = DiaphragmPumpDispenseVolumeOp.from_args(target_sample=lot.batches[0].samples[0],
                                                       liquid_name='water',
                                                       dispense_volume=100,
                                                       dispense_unit="mL")
        self.assertIsNotNone(t_op.object_id)
        self.assertEqual(t_op.liquid_name, "water")
        self.assertEqual(t_op.dispense_volume, 100)
        self.assertEqual(t_op.dispense_unit, "mL")

        station.add_station_op(t_op)
        station.update_assigned_op()
        op_result = MaterialOpResult.from_args(t_op.object_id,
                                               [t_op.liquid_name],
                                               [t_op.dispense_volume],
                                               [t_op.dispense_unit])

        station.complete_assigned_op(OpOutcome.SUCCEEDED, [op_result])
        self.assertEqual(station.liquids_dict["water"].volume, 300)
        self.assertEqual(station.liquids_dict["water"].volume_unit, "mL")

    def test_sim_handler(self):
        station = DiaphragmPumpStation.from_dict(self.station_dict)
        batch_1 = Batch.from_args(3)
        lot = Lot.from_args([batch_1])

        # add batches to station
        station.add_lot(lot)

        # construct handler
        handler = SimDiaphragmPumpStationHandler(station)

        # initialise the handler
        self.assertTrue(handler.initialise())

        # test handling DiaphragmPumpDispenseVolumeOp
        t_op = DiaphragmPumpDispenseVolumeOp.from_args(target_sample=lot.batches[0].samples[0],
                                                       liquid_name='water',
                                                       dispense_volume=100,
                                                       dispense_unit="mL")
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
