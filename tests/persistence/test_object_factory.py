import unittest
from mongoengine import connect

from archemist.core.persistence.object_factory import RobotFactory, RobotOpFactory, StationFactory,\
                                                        StationOpFactory, ProcessFactory
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.state.robot_op import RobotOpDescriptor
from archemist.core.state.lot import Lot, Batch
from archemist.core.util.location import Location

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
            'total_batch_capacity': 2,
            'process_batch_capacity': 1,
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

        # TODO test generic handler

        # TODO test station from archemist.stations

    def test_station_op_factory(self):
        # general station op
        station_op = StationOpDescriptor.construct_op()

        # test construction from model
        station_op_from_model = StationOpFactory.create_from_model(station_op.model)
        self.assertEqual(station_op.uuid, station_op_from_model.uuid)

        # test construction from object id
        station_op_from_object_id = StationOpFactory.create_from_object_id(station_op.object_id)
        self.assertEqual(station_op.uuid, station_op_from_object_id.uuid)

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

        # TODO test generic handler

        # TODO test robot from archemist.robots

    def test_robot_op_factory(self):
        # general robot op
        robot_op = RobotOpDescriptor.construct_op()

        # test construction from model
        robot_op_from_model = RobotOpFactory.create_from_model(robot_op.model)
        self.assertEqual(robot_op.uuid, robot_op_from_model.uuid)

        # TODO test robot_op from archemist.robots

    def test_process_factory(self):
        # general station process
        # construct from args
        batch_1 = Batch.from_arguments(3, Location(1, 2, "some_frame"))
        lot = Lot.from_args([batch_1])
        proc_from_args = ProcessFactory.create_from_args("StationProcess",lot, {})
        self.assertIsNotNone(proc_from_args.uuid)

        # construct from model
        proc_from_model  = ProcessFactory.create_from_model(proc_from_args.model)
        self.assertEqual(proc_from_args.uuid, proc_from_model.uuid)

        # construct from objectid
        proc_from_object_id = ProcessFactory.create_from_object_id(proc_from_args.object_id)
        self.assertEqual(proc_from_args.uuid, proc_from_object_id.uuid)

        # TODO test station_process from archemist.stations

