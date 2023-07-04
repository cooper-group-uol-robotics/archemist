import unittest

from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.util.location import Location
from mongoengine import connect

class LotTest(unittest.TestCase):
    def setUp(self):
        connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')

    def test_lot(self):
        batch_1 = Batch.from_arguments(3, Location(1, 2, "some_frame"))
        batch_2 = Batch.from_arguments(3, Location(1, 2, "some_frame"))
        lot = Lot.from_args([batch_1, batch_2])
        self.assertIsNotNone(lot.uuid)
        self.assertEqual(len(lot.batches), 2)
        self.assertEqual(lot.batches[0].uuid, batch_1.uuid)
        self.assertEqual(lot.batches[1].uuid, batch_2.uuid)

if __name__ == "__main__":
    unittest.main()