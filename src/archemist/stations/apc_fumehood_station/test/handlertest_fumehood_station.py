#!/usr/bin/env python3

import unittest
from archemist.stations.apc_fumehood_station.state import (
    APCCartridge, APCFumehoodStation, APCDispenseSolidOp, APCCloseSashOp, APCOpenSashOp
)

from archemist.core.state.material import Liquid

from archemist.stations.apc_fumehood_station.handler import APCFumeHoodStationRosHandler
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.util.enums import StationState, OpOutcome
from archemist.core.state.station_op_result import ProcessOpResult, MaterialOpResult
from archemist.core.persistence.models_proxy import ModelProxy, ListProxy, DictProxy
from archemist.stations.apc_fumehood_station.handler import SimAPCFumehoodStationHandler

from archemist.core.state.robot_op import RobotTaskOp, RobotWaitOp
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.util.enums import OpOutcome, ProcessStatus

from datetime import datetime, date

from mongoengine import connect

print("importing done")


class APCFumeHoodStationHandlerTest(unittest.TestCase):
    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        self.station_doc = {
            'type': 'APCFumehoodStation',
            'id': 21,
            'location': {'coordinates': [1,7], 'descriptor': "APCFumehoodStation"},
            'total_lot_capacity': 1,
            'handler': 'SimStationOpHandler',
            'properties': {
                'cartridges': [
                    {'associated_solid': "NaCl", 'hotel_index': 1},
                    {'associated_solid': "NaCl", 'hotel_index': 2}
                    ]
            },
            'materials':
            {
                'solids': [{
                    'name': 'NaCl',
                    'amount': 100,
                    'unit': 'mg',
                    'expiry_date': date.fromisoformat('2025-02-11'),
                    'details': None
                }]
            }
        }

    def test_handler(self):
        station = APCFumehoodStation.from_dict(self.station_doc)
        self.assertIsNotNone(station)
        self.assertEqual(station.state, StationState.INACTIVE)
            # construct lot
        batch = Batch.from_args(1)
        lot = Lot.from_args([batch])

        # add lot to station
        station.add_lot(lot)

        # construct handler
        handler = APCFumeHoodStationRosHandler(station)
        # initialise handler
        self.assertTrue(handler.initialise())

        # test IKAHeatStirBatchOp
        # construct op
        sample = lot.batches[0].samples[0]
        # t_op = APCCloseSashOp.from_args()
        t_op = APCOpenSashOp.from_args()

        self.assertIsNotNone(t_op.object_id)

        station.add_station_op(t_op)
        station.update_assigned_op()
        self.assertEqual(station.assigned_op, t_op)


        # execute op
        handler.execute_op()

        # wait for results
        handler.is_op_execution_complete()

        self.assertEqual(handler.is_op_execution_complete(), False)

        handler._op_complete = True

        self.assertEqual(handler.is_op_execution_complete(), True)



        outcome, op_results = handler.get_op_result()


        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        # self.assertIsInstance(op_results[0], ProcessOpResult)
        self.assertEqual(op_results, None)



if __name__ == '__main__':
    unittest.main()
