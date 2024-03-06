import unittest

from mongoengine import connect

from archemist.stations.fisher_balance_station.state import (FisherWeightingStation,
                                                             FisherWeighOp,
                                                             FisherWeighResult)
from archemist.core.util.enums import StationState
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot


class FisherBalanceStationTest(unittest.TestCase):
    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        self.station_doc = {
            'type': 'FisherWeightingStation',
            'id': 20,
            'location': {'coordinates': [1, 7], 'descriptor': "FisherWeightingStation"},
            'total_lot_capacity': 1,
            'handler': 'SimStationOpHandler',
            'properties': None,
            'materials': None
        }

        self.station = FisherWeightingStation.from_dict(self.station_doc)

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

        # test station op construction
        t_op = FisherWeighOp.from_args(target_sample=lot.batches[0].samples[0])
        self.assertIsNotNone(t_op.object_id)

        # test station op result
        t_result_op = FisherWeighResult.from_args(origin_op=t_op.object_id,
                                                  reading_value=42.1,
                                                  unit="mg")
        self.assertIsNotNone(t_result_op.object_id)
        self.assertEqual(t_result_op.reading_value, 42.1)
        self.assertEqual(t_result_op.unit, "mg")


if __name__ == '__main__':
    unittest.main()
