#!/usr/bin/env python3

import unittest

from mongoengine import connect
from archemist.stations.apc_filtration_station.state import (
    APCFiltrationStation, 
    APCFilterProductOp, 
    APCDrainWasteOp, 
    APCDryProductOp)
from archemist.stations.apc_filtration_station.handler import SimAPCFiltrationStationHandler
from archemist.core.state.station_op_result import ProcessOpResult
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.util.enums import OpOutcome


class APCWeighingStationTest(unittest.TestCase):

    def setUp(self) -> None:
        self._db_name = 'archemist_test'
        self._client = connect(
            db=self._db_name, alias='archemist_state', host='mongodb://localhost:27017',
            uuidrepresentation='standard'
        )

        self.station_doc = {
            'type': 'APCFiltrationStation',
            'id': 35,
            'location': {'coordinates': [1,7], 'descriptor': "APCFiltrationStation"},
            'total_lot_capacity': 1,
            'handler': 'SimStationOpHandler',
            'properties': None,
            'materials': None
        }

    def tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_state(self):
        # test station is constructed properly
        station = APCFiltrationStation.from_dict(self.station_doc)
        self.assertIsNotNone(station)

        # construct lot and add it to station
        batch = Batch.from_args(1)
        lot = Lot.from_args([batch])
        station.add_lot(lot)

        # test APCFilterProductOp
        t_op = APCFilterProductOp.from_args()
        self.assertIsNotNone(t_op.object_id)
        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)

        # test APCDrainWasteOp
        t_op = APCDrainWasteOp.from_args()
        self.assertIsNotNone(t_op.object_id)
        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)

        # test APCDryProductOp
        t_op = APCDryProductOp.from_args(target_sample=batch.samples[0],
                                         duration=3,
                                         time_unit="minute")
        self.assertIsNotNone(t_op.object_id)
        self.assertEqual(t_op.duration, 3)
        self.assertEqual(t_op.time_unit, "minute")
        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)

    def test_sim_handler(self):
        station = APCFiltrationStation.from_dict(self.station_doc)
        batch = Batch.from_args(1)
        lot = Lot.from_args([batch])

        # add batches to station
        station.add_lot(lot)

        # construct handler
        handler = SimAPCFiltrationStationHandler(station)

        # initialise the handler
        self.assertTrue(handler.initialise())

        # construct weigh result op
        t_op = APCDryProductOp.from_args(target_sample=batch.samples[0],
                                         duration=3,
                                         time_unit="minute")
        station.add_station_op(t_op)
        station.update_assigned_op()
        
        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertIsInstance(op_results[0], ProcessOpResult)
        self.assertDictEqual(dict(op_results[0].parameters), {"duration": 3, "time_unit": "minute"})


if __name__ == '__main__':
    unittest.main()