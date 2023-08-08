import unittest
from datetime import datetime
import uuid
from bson.objectid import ObjectId
from mongoengine import connect

from archemist.core.state.station_op import StationOpDescriptor, StationOpDescriptorModel

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
        station_op = StationOpDescriptor.construct_op()
        self.assertEqual(station_op.associated_station, "Station")
        self.assertIsNotNone(station_op.uuid)
        self.assertIsNone(station_op.requested_by)
        dummy_object_id = ObjectId.from_datetime(datetime.now())
        station_op.requested_by = dummy_object_id
        self.assertEqual(station_op.requested_by, dummy_object_id)
        self.assertFalse(station_op.has_result)
        self.assertFalse(station_op.was_successful)
        self.assertIsNone(station_op.start_timestamp)
        self.assertIsNone(station_op.end_timestamp)
        
        # test start timestamp
        station_op.add_start_timestamp()
        self.assertIsNotNone(station_op.start_timestamp)
        self.assertLessEqual(station_op.start_timestamp, datetime.now())
        start_timestamp = station_op.start_timestamp
        station_op.start_timestamp = datetime.now()
        self.assertGreater(station_op.start_timestamp, start_timestamp)

        # test end timestamp
        station_op.complete_op(True)
        self.assertTrue(station_op.has_result)
        self.assertTrue(station_op.was_successful)
        self.assertIsNotNone(station_op.end_timestamp)
        self.assertLessEqual(station_op.end_timestamp, datetime.now())
        self.assertGreater(station_op.end_timestamp, station_op.start_timestamp)
        end_timestamp = station_op.end_timestamp
        station_op.end_timestamp = datetime.now()
        self.assertGreater(station_op.end_timestamp, end_timestamp)

if __name__ == "__main__":
    unittest.main()