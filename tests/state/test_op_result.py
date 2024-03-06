import unittest
from bson.objectid import ObjectId
from mongoengine import connect

from archemist.core.state.station_op_result import StationOpResult, MaterialOpResult, ProcessOpResult


class OpResultTest(unittest.TestCase):
    def setUp(self) -> None:
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

    def tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_op_result(self):
        mock_origin_op = ObjectId()
        op_result = StationOpResult.from_args(origin_op=mock_origin_op)
        self.assertIsNotNone(op_result)
        self.assertIsNotNone(op_result.object_id)
        self.assertEqual(op_result.origin_op, mock_origin_op)

    def test_material_op_result(self):
        mock_origin_op = ObjectId()
        op_result = MaterialOpResult.from_args(origin_op=mock_origin_op,
                                               material_names=["water"],
                                               amounts=[1.1],
                                               units=["mg"])
        self.assertIsNotNone(op_result)
        self.assertEqual(op_result.origin_op, mock_origin_op)
        self.assertEqual(op_result.material_names[0], "water")
        self.assertEqual(op_result.amounts[0], 1.1)
        self.assertEqual(op_result.units[0], "mg")

    def test_process_op_result(self):
        mock_origin_op = ObjectId()
        parameters_dict = {"operation": "stir", "speed": 500}
        op_result = ProcessOpResult.from_args(origin_op=mock_origin_op,
                                              parameters=parameters_dict)
        self.assertIsNotNone(op_result)
        self.assertEqual(op_result.origin_op, mock_origin_op)
        self.assertDictEqual(op_result.parameters, parameters_dict)
