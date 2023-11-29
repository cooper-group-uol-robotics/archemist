import unittest

from mongoengine import connect
from archemist.stations.apc_weighing_station.state import (
    APCWeighingStation, 
    APCOpenBalanceDoorOp, 
    APCCloseBalanceDoorOp,
    APCTareOp,
    APCWeighingOp,
    APCWeighResult
)
from archemist.stations.apc_weighing_station.handler import SimAPCWeighingStationHandler
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.util.enums import OpOutcome


class APCWeighingStationTest(unittest.TestCase):

    def setUp(self) -> None:
        self._db_name = 'archemist_test'
        self._client = connect(
            db=self._db_name, alias='archemist_state', host='mongodb://localhost:27017')

        self.station_doc = {
            'type': 'APCWeighingStation',
            'id': 35,
            'location': {'coordinates': [1,7], 'descriptor': "ApcWeighingStation"},
            'total_lot_capacity': 1,
            'handler': 'SimStationOpHandler',
            'properties': 
            {
                'funnel_storage_capacity': 3
            },
            'materials': None
        }

        self.station = APCWeighingStation.from_dict(self.station_doc)

    def tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_state(self):
        # test station is constructed properly
        self.assertIsNotNone(self.station)

        self.assertFalse(self.station.balance_doors_open)
        self.assertEqual(self.station.funnel_storage_capacity, 3)
        self.assertEqual(self.station.funnel_storage_index, 0)
        self.station.funnel_storage_index += 1
        self.assertEqual(self.station.funnel_storage_index, 1)

        # construct lot and add it to station
        batch = Batch.from_args(1)
        lot = Lot.from_args([batch])
        self.station.add_lot(lot)

        # test APCOpenBalanceDoorOp
        t_op = APCOpenBalanceDoorOp.from_args()
        self.assertIsNotNone(t_op.object_id)
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertTrue(self.station.balance_doors_open)

        # test APCCloseBalanceDoorOp
        t_op = APCCloseBalanceDoorOp.from_args()
        self.assertIsNotNone(t_op.object_id)
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertFalse(self.station.balance_doors_open)

        # test TareOp
        t_op = APCTareOp.from_args()
        self.assertIsNotNone(t_op.object_id)

        # test WeighingOp
        t_op = APCWeighingOp.from_args(target_sample=batch.samples[0])
        self.assertIsNotNone(t_op.object_id)

        # test WeighingOpResult
        t_result_op = APCWeighResult.from_args(
            origin_op=t_op.object_id,
            reading_value=42.1)
        self.assertIsNotNone(t_result_op.object_id)
        self.assertEqual(t_result_op.reading_value, 42.1)
        self.assertEqual(t_result_op.unit, "g")

    def test_sim_handler(self): 
        batch = Batch.from_args(1)
        lot = Lot.from_args([batch])

        # add batches to station
        self.station.add_lot(lot)

        # construct handler
        handler = SimAPCWeighingStationHandler(self.station)

        # initialise the handler
        self.assertTrue(handler.initialise())

        # construct weigh result op
        t_op = APCWeighingOp.from_args(target_sample=batch.samples[0])
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        
        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertIsInstance(op_results[0], APCWeighResult)
        self.assertEqual(op_results[0].reading_value, 42)


if __name__ == '__main__':
    unittest.main()
