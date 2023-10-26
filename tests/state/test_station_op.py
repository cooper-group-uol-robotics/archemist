import unittest
from datetime import datetime
from bson.objectid import ObjectId
from mongoengine import connect

from archemist.core.state.station_op import (StationOpDescriptor,
                                             StationLotOpDescriptor,
                                             StationBatchOpDescriptor,
                                             StationSampleOpDescriptor,
                                             OpOutcome,
                                             OpResult)
from archemist.core.state.lot import Lot, Batch
from archemist.core.state.sample import Sample

class StationOpTest(unittest.TestCase):
    def setUp(self) -> None:
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

    def  tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_station_op(self):
        # construct op
        station_op = StationOpDescriptor.from_args()
        self.assertEqual(station_op.associated_station, "Station")
        self.assertIsNotNone(station_op.object_id)
        self.assertIsNone(station_op.requested_by)
        dummy_object_id = ObjectId.from_datetime(datetime.now())
        station_op.requested_by = dummy_object_id
        self.assertEqual(station_op.requested_by, dummy_object_id)
        self.assertIsNone(station_op.outcome)
        self.assertIsNone(station_op.start_timestamp)
        self.assertIsNone(station_op.end_timestamp)
        
        # test start timestamp
        station_op.add_start_timestamp()
        self.assertIsNotNone(station_op.start_timestamp)
        self.assertLessEqual(station_op.start_timestamp, datetime.now())
        start_timestamp = station_op.start_timestamp
        station_op.start_timestamp = datetime.now()
        self.assertGreater(station_op.start_timestamp, start_timestamp)

        # test op completion
        results = []
        for _ in range(2):
            results.append(OpResult.from_args(origin_op=station_op.object_id))
        
        station_op.complete_op(OpOutcome.SUCCEEDED, results)
        self.assertEqual(station_op.outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(station_op.results), 2)
        self.assertIsNotNone(station_op.end_timestamp)
        self.assertLessEqual(station_op.end_timestamp, datetime.now())
        self.assertGreater(station_op.end_timestamp, station_op.start_timestamp)
        end_timestamp = station_op.end_timestamp
        station_op.end_timestamp = datetime.now()
        self.assertGreater(station_op.end_timestamp, end_timestamp)

    def test_station_lot_op(self):
        num_samples = 2
        batch_1 = Batch.from_args(num_samples)
        batch_2 = Batch.from_args(num_samples)
        lot = Lot.from_args([batch_1, batch_2])
        # test op
        station_op_1 = StationLotOpDescriptor.from_args(target_lot=lot)
        self.assertEqual(station_op_1.target_lot, lot)

        # test op complete with a single result
        results = [OpResult.from_args(origin_op=station_op_1.object_id)]
        station_op_1.complete_op(OpOutcome.SUCCEEDED, results)
        self.assertEqual(station_op_1.outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(station_op_1.results), 1)
        self.assertIsNotNone(station_op_1.end_timestamp)

        for batch in lot.batches:
            for sample in batch.samples:
                self.assertEqual(sample.result_ops[0], results[0])

        # test op complete with result per batch
        station_op_2 = StationLotOpDescriptor.from_args(target_lot=lot)
        results = []
        for _ in range(lot.num_batches):
            results.append(OpResult.from_args(origin_op=station_op_2.object_id))
        
        station_op_2.complete_op(OpOutcome.SUCCEEDED, results)
        self.assertEqual(station_op_2.outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(station_op_2.results), 2)
        self.assertIsNotNone(station_op_2.end_timestamp)

        for index, batch in enumerate(lot.batches):
            for sample in batch.samples:
                self.assertEqual(sample.result_ops[1], results[index])

        # test op complete with result per sample
        station_op_3 = StationLotOpDescriptor.from_args(target_lot=lot)
        results = []
        for _ in range(lot.num_batches*num_samples):
            results.append(OpResult.from_args(origin_op=station_op_3.object_id))
        
        station_op_3.complete_op(OpOutcome.SUCCEEDED, results)
        self.assertEqual(station_op_3.outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(station_op_3.results), lot.num_batches*num_samples)
        self.assertIsNotNone(station_op_3.end_timestamp)

        index = 0
        for batch in lot.batches:
            for sample in batch.samples:
                self.assertEqual(sample.result_ops[2], results[index])
                index += 1

    def test_station_batch_op(self):
        num_samples = 2
        batch = Batch.from_args(num_samples)
        # test op
        station_op_1 = StationBatchOpDescriptor.from_args(target_batch=batch)
        self.assertEqual(station_op_1.target_batch, batch)

        # test op complete with a single result
        results = [OpResult.from_args(origin_op=station_op_1.object_id)]
        station_op_1.complete_op(OpOutcome.SUCCEEDED, results)
        self.assertEqual(station_op_1.outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(station_op_1.results), 1)
        self.assertIsNotNone(station_op_1.end_timestamp)

        for sample in batch.samples:
            self.assertEqual(sample.result_ops[0], results[0])

        # test op complete with result per sample
        station_op_2 = StationBatchOpDescriptor.from_args(target_batch=batch)
        results = []
        for _ in range(num_samples):
            results.append(OpResult.from_args(origin_op=station_op_2.object_id))
        
        station_op_2.complete_op(OpOutcome.SUCCEEDED, results)
        self.assertEqual(station_op_2.outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(station_op_2.results), 2)
        self.assertIsNotNone(station_op_2.end_timestamp)

        for index, sample in enumerate(batch.samples):
            self.assertEqual(sample.result_ops[1], results[index])

    def test_station_sample_op(self):
        sample = Sample.from_args(parent_batch_id=ObjectId())
        # test op
        station_op = StationSampleOpDescriptor.from_args(target_sample=sample)
        self.assertEqual(station_op.target_sample, sample)

        # test op complete with a single result
        results = [OpResult.from_args(origin_op=station_op.object_id)]
        station_op.complete_op(OpOutcome.SUCCEEDED, results)
        self.assertEqual(station_op.outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(station_op.results), 1)
        self.assertIsNotNone(station_op.end_timestamp)

        self.assertEqual(sample.result_ops[0], results[0])

if __name__ == "__main__":
    unittest.main()