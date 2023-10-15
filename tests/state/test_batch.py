import unittest
from bson.objectid import ObjectId
from archemist.core.state.batch import Batch
from archemist.stations.ika_digital_plate_station.state import IKAHeatingOpDescriptor, IKAStirringOpDescriptor
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
        # test uuid and location
        batch = Batch.from_args(2,Location(1,3,'table_frame'))
        self.assertIsNotNone(batch.uuid)
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

        # test num_samples
        self.assertEqual(batch.num_samples, 2)

    def test_batch_samples_field(self):
        batch = Batch.from_args(2,Location(1,3,'table_frame'))
        samples = batch.samples

        for sample in samples:
            self.assertEqual(len(sample.materials), 0)
            self.assertEqual(len(sample.details), 0)
            self.assertEqual(len(sample.station_ops), 0)

            # test materials
            sample.add_material("water", 123, 5.3, "ml")
            sample.add_material("salt", 456, 1.1, "mg")
            self.assertEqual(len(sample.materials), 2)
            
            added_material_1 = sample.materials[0]
            self.assertEqual(added_material_1["name"], "water")
            self.assertEqual(added_material_1["id"], 123)
            self.assertEqual(added_material_1["amount"], 5.3)
            self.assertEqual(added_material_1["unit"], "ml")

            added_material_2 = sample.materials[1]
            self.assertEqual(added_material_2["name"], "salt")
            self.assertEqual(added_material_2["id"], 456)
            self.assertEqual(added_material_2["amount"], 1.1)
            self.assertEqual(added_material_2["unit"], "mg")

            # test details
            sample.details = {"capped": False, "cap_color": "blue"}
            self.assertEqual(len(sample.details), 2)
            self.assertFalse(sample.details["capped"])
            self.assertEqual(sample.details["cap_color"], "blue")
            sample.details["capped"] = True
            self.assertTrue(sample.details["capped"])

            # test station_ops
            ika_op1 = IKAHeatingOpDescriptor.from_args(temperature=50, duration=10)
            ika_op2 = IKAStirringOpDescriptor.from_args(stirring_speed=70, duration=10)

            sample.add_station_op(ika_op1)
            sample.add_station_op(ika_op2)
            self.assertEqual(len(sample.station_ops), 2)
            op1 = sample.station_ops[0]
            self.assertTrue(isinstance(op1, IKAHeatingOpDescriptor))
            self.assertEqual(op1.target_temperature, 50)
            self.assertEqual(op1.target_duration, 10)
            op2 = sample.station_ops[1]
            self.assertTrue(isinstance(op2, IKAStirringOpDescriptor))
            self.assertEqual(op2.target_stirring_speed, 70)

if __name__ == '__main__':
    unittest.main()