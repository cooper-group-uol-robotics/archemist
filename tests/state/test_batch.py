import unittest
from bson.objectid import ObjectId
from archemist.core.state.batch import Batch
from mongoengine import connect
from archemist.core.util.location import Location

class BatchTest(unittest.TestCase):
    def setUp(self) -> None:
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

    def  tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_batch_general_fields(self):
        # test object_id and location
        batch = Batch.from_args(2,Location(1,3,'table_frame'))
        self.assertIsNotNone(batch.object_id)
        self.assertEqual(batch.location, Location(1,3,'table_frame'))
        batch.location = Location(1,3,'chair_frame')
        self.assertEqual(batch.location, Location(1,3,'chair_frame'))
        self.assertIsNone(batch.parent_lot_id)
        lot_id = ObjectId()
        batch.parent_lot_id = lot_id
        self.assertEqual(batch.parent_lot_id, lot_id)

        # test station_stamps
        self.assertEqual(len(batch.station_stamps), 0)
        batch.add_station_stamp("SomeStation_123")
        self.assertEqual(len(batch.station_stamps), 1)
        self.assertTrue("SomeStation_123" in batch.station_stamps[0])

        # test samples
        self.assertEqual(batch.num_samples, len(batch.samples))
        self.assertEqual(batch.samples[0].parent_batch_id, batch.object_id)

if __name__ == '__main__':
    unittest.main()