import unittest
from bson.objectid import ObjectId
from archemist.core.state.sample import Sample
from archemist.core.state.op_result import MaterialOpResult, ProcessOpResult
from mongoengine import connect

class BatchTest(unittest.TestCase):
    def setUp(self) -> None:
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

    def  tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_sample_field(self):
        parent_id = ObjectId()
        sample = Sample.from_args(parent_id)

        self.assertEqual(sample.parent_batch_id, parent_id)
        self.assertFalse(sample.materials)
        self.assertFalse(sample.result_ops)

        # test adding MaterialOpResult
        mock_origin_op = ObjectId()
        material_op_1 = MaterialOpResult.from_args(origin_op=mock_origin_op,
                                               material_name="water",
                                               material_id=12,
                                               amount=1.1,
                                               unit="mg")
        sample.add_result_op(material_op_1)
        self.assertEqual(len(sample.result_ops), 1)
        self.assertEqual(sample.result_ops[0], material_op_1)
        self.assertEqual(len(sample.materials), 1)
        self.assertDictEqual(dict(sample.materials), {"water": {"amount": 1.1, "unit": "mg"}})

        material_op_2 = MaterialOpResult.from_args(origin_op=mock_origin_op,
                                               material_name="water",
                                               material_id=12,
                                               amount=0.9,
                                               unit="mg")
        
        sample.add_result_op(material_op_2)
        self.assertEqual(len(sample.result_ops), 2)
        self.assertEqual(sample.result_ops[1], material_op_2)
        self.assertEqual(len(sample.materials), 1)
        self.assertDictEqual(dict(sample.materials), {"water": {"amount": 2.0, "unit": "mg"}})

        # test adding ProcessOpResult
        parameters_dict = {"operation": "stir", "speed": 500}
        process_op_result = ProcessOpResult.from_args(origin_op=mock_origin_op,
                                              parameters=parameters_dict)
        
        sample.add_result_op(process_op_result)
        self.assertEqual(len(sample.result_ops), 3)
        self.assertEqual(sample.result_ops[2], process_op_result)
        self.assertEqual(len(sample.materials), 1)
        
        

if __name__ == '__main__':
    unittest.main()