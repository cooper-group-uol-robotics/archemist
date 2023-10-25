import unittest
from mongoengine import connect

from archemist.core.persistence.object_factory import RobotFactory, RobotOpFactory, StationFactory,\
                                                        StationOpFactory, ProcessFactory, OpResultFactory
from archemist.core.state.lot import Lot, Batch
from archemist.core.processing.handler import SimStationOpHandler, SimRobotOpHandler
from archemist.core.state.op_result import OpResult, ProcessOpResult
from archemist.core.util.location import Location
from bson.objectid import ObjectId


class ObjectFactoryTest(unittest.TestCase):
    def setUp(self) -> None:
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

    def  tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_station_factory(self):
        station_dict = {
            'type': 'Station',
            'id': 23,
            'location': {'node_id': 1, 'graph_id': 7},
            'batch_capacity': 2,
            'handler': 'GenericStationHandler',
            'total_lot_capacity': 2,
        }

        # general station
        # test construction from dict
        station_from_dict =  StationFactory.create_from_dict(station_dict)
        self.assertEqual(station_from_dict.id, 23)

        # station_dict['type'] = "NonExistantStation"
        # with self.assertRaises(NameError):
        #     StationFactory.create_from_dict(station_dict)

        # test construction from model
        station_from_model = StationFactory.create_from_model(station_from_dict.model)
        self.assertEqual(station_from_model.id, 23)

        # test construction from object id
        station_from_object_id = StationFactory.create_from_object_id(station_from_dict.object_id)
        self.assertEqual(station_from_object_id.id, 23)

        # test sim station op handler
        sim_op_handler = StationFactory.create_op_handler(station_from_dict, use_sim_handler=True)
        self.assertIsNotNone(sim_op_handler)
        self.assertTrue(isinstance(sim_op_handler, SimStationOpHandler))

        # TODO test station from archemist.stations

    def test_station_op_factory(self):
        # test construction from args
        station_op_from_args = StationOpFactory.create_from_args(op_type="StationOpDescriptor")
        self.assertIsNotNone(station_op_from_args)

        # test construction from model
        station_op_from_model = StationOpFactory.create_from_model(station_op_from_args.model)
        self.assertEqual(station_op_from_args, station_op_from_model)

        # test construction from object id
        station_op_from_object_id = StationOpFactory.create_from_object_id(station_op_from_args.object_id)
        self.assertEqual(station_op_from_args, station_op_from_object_id)

        # TODO test station_op from archemist.stations

    def test_op_result_factory(self):
        # construct result_op 
        op_result = OpResult.from_args(origin_op=ObjectId())
        
        # test construction from model
        op_result_from_model = OpResultFactory.create_from_model(op_result.model)
        self.assertEqual(op_result, op_result_from_model)

        # test construction from object id
        op_result_from_object_id = OpResultFactory.create_from_object_id(op_result.object_id)
        self.assertEqual(op_result, op_result_from_object_id)

        # construct process_op
        parameters_dict = {"speed": 500} 
        process_op_result = ProcessOpResult.from_args(origin_op=ObjectId(), parameters=parameters_dict)
        
        # test construction from model
        op_result_from_model = OpResultFactory.create_from_model(process_op_result.model)
        self.assertEqual(process_op_result, op_result_from_model)

        # test construction from object id
        op_result_from_object_id = OpResultFactory.create_from_object_id(process_op_result.object_id)
        self.assertEqual(process_op_result, op_result_from_object_id)

        # TODO test station_op from archemist.stations

    def test_robot_factory(self):
        robot_dict = {
            "type": "Robot",
            "location": {"node_id": 1, "graph_id": 2, "frame_name": "a_frame"},
            "id": 187,
            "batch_capacity":2,
            "handler": "GenericRobotHandler"
        }

        # general robot
        # test construction from dict
        robot_from_dict =  RobotFactory.create_from_dict(robot_dict)
        self.assertEqual(robot_from_dict.id, 187)

        robot_dict['type'] = "NonExistantRobot"
        with self.assertRaises(NameError):
            RobotFactory.create_from_dict(robot_dict)

        # test construction from model
        robot_from_model = RobotFactory.create_from_model(robot_from_dict.model)
        self.assertEqual(robot_from_model.id, 187)

        # test construction from object id
        robot_from_object_id = RobotFactory.create_from_object_id(robot_from_dict.object_id)
        self.assertEqual(robot_from_object_id.id, 187)

        # test sim robot op handler
        sim_op_handler = RobotFactory.create_op_handler(robot_from_dict, use_sim_handler=True)
        self.assertIsNotNone(sim_op_handler)
        self.assertTrue(isinstance(sim_op_handler, SimRobotOpHandler))

        # TODO test robot from archemist.robots

    def test_robot_op_factory(self):
        # test construction from args
        robot_op_from_args = RobotOpFactory.create_from_args("RobotOpDescriptor")
        self.assertIsNotNone(robot_op_from_args)

        # test construction from model
        robot_op_from_model = RobotOpFactory.create_from_model(robot_op_from_args.model)
        self.assertEqual(robot_op_from_args, robot_op_from_model)

        # test construction from args
        random_station_id = ObjectId()
        op_params = {
            "name": "some_random_task",
            "target_robot": "Robot",
            "origin_station_id": random_station_id,
            "params": {"index": 1}
            }
        robot_op_from_args = RobotOpFactory.create_from_args("RobotTaskOpDescriptor", op_params)
        
        self.assertIsNotNone(robot_op_from_args)
        self.assertEqual(len(robot_op_from_args.params), 1)
        self.assertEqual(robot_op_from_args.params["index"], 1)
        self.assertEqual(robot_op_from_args.requested_by, random_station_id)

        # test construction from model
        robot_op_from_model = RobotOpFactory.create_from_model(robot_op_from_args.model)
        self.assertEqual(robot_op_from_args, robot_op_from_model)

        # TODO test robot_op from archemist.robots

    def test_process_factory(self):
        # general station process
        proc_dict = {"type": "StationProcess"}
        # construct from dict
        batch_1 = Batch.from_args(3, Location(1, 2, "some_frame"))
        lot = Lot.from_args([batch_1])
        proc_from_args = ProcessFactory.create_from_dict(proc_dict,lot)
        self.assertIsNotNone(proc_from_args)

        # construct from model
        proc_from_model  = ProcessFactory.create_from_model(proc_from_args.model)
        self.assertEqual(proc_from_args.object_id, proc_from_model.object_id)

        # construct from objectid
        proc_from_object_id = ProcessFactory.create_from_object_id(proc_from_args.object_id)
        self.assertEqual(proc_from_args.object_id, proc_from_object_id.object_id)

        # TODO test station_process from archemist.stations

if __name__ == "__main__":
    unittest.main()