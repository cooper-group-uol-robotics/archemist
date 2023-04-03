import unittest
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaLBRMaintenanceTask
from archemist.stations.ika_digital_plate_station.state import (
    IKAHeatingOpDescriptor,
    IKAStirringOpDescriptor,
)
from archemist.core.state.batch import Batch
from archemist.core.models.batch_model import BatchModel
from archemist.core.models.robot_op_model import RobotOpDescriptorModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from archemist.core.util.location import Location
from archemist.core.util.list_field_adapter import OpListAdapter, StateObjListAdapter
from archemist.core.persistence.object_factory import StationFactory, RobotFactory
from mongoengine import connect, Document, fields


class TestModel(Document):
    batches = fields.ListField(fields.ReferenceField(BatchModel), default=[])
    robot_ops = fields.EmbeddedDocumentListField(RobotOpDescriptorModel, default=[])
    station_ops = fields.EmbeddedDocumentListField(StationOpDescriptorModel, default=[])

    meta = {"db_alias": "archemist_state"}


class ListAdapterTest(unittest.TestCase):
    def test_op_list(self):
        rob_op1 = KukaLBRTask.from_args("op1")
        rob_op2 = KukaLBRMaintenanceTask.from_args("op2", [])
        rob_op3 = KukaLBRTask.from_args("op3")
        robot_op_models = [rob_op1.model, rob_op2.model, rob_op3.model]

        station_op1 = IKAHeatingOpDescriptor.from_args(temperature=200, duration=15)
        station_op2 = IKAStirringOpDescriptor.from_args(stirring_speed=210, duration=10)
        station_op3 = IKAHeatingOpDescriptor.from_args(temperature=100, duration=3)
        station_op_models = [station_op1.model, station_op2.model]

        batch1 = Batch.from_arguments(batch_id=1, num_samples=2, location=Location())
        batch2 = Batch.from_arguments(batch_id=2, num_samples=6, location=Location())
        batch3 = Batch.from_arguments(batch_id=3, num_samples=6, location=Location())

        batch_models = [batch1.model, batch2.model]

        test_model = TestModel(
            robot_ops=robot_op_models,
            station_ops=station_op_models,
            batches=batch_models,
        )
        test_model.save()

        """ robot_op list adapter """
        robot_ops_list = OpListAdapter(test_model, "robot_ops", RobotFactory)

        # len operation
        self.assertEqual(len(robot_ops_list), 3)

        # get operation
        self.assertEqual(robot_ops_list[0].name, rob_op1.name)

        # set operation
        robot_ops_list[0] = rob_op3
        self.assertEqual(robot_ops_list[0].name, rob_op3.name)

        # append operation
        robot_ops_list.append(rob_op1)
        self.assertEqual(len(robot_ops_list), 4)
        self.assertEqual(robot_ops_list[-1].name, rob_op1.name)

        # pop operation
        rpop_op = robot_ops_list.pop()
        self.assertEqual(len(robot_ops_list), 3)
        self.assertEqual(rpop_op.name, rob_op1.name)
        lpop_op = robot_ops_list.popleft()
        self.assertEqual(len(robot_ops_list), 2)
        self.assertEqual(lpop_op.name, rob_op3.name)

        # extend operation
        robot_ops_list.extend([rob_op1, rob_op2])
        self.assertEqual(len(robot_ops_list), 4)
        self.assertEqual(robot_ops_list[0].name, rob_op2.name)
        self.assertEqual(robot_ops_list[1].name, rob_op3.name)
        self.assertEqual(robot_ops_list[2].name, rob_op1.name)

        # iteration operation
        i = 0
        indices = [2, 3, 1, 2]
        for op in robot_ops_list:
            self.assertEqual(op.name, f"op{indices[i]}")
            i += 1

        # remove operation
        robot_ops_list.remove(rob_op1)
        self.assertEqual(len(robot_ops_list), 3)
        self.assertEqual(robot_ops_list[0].name, rob_op2.name)
        self.assertEqual(robot_ops_list[1].name, rob_op3.name)
        self.assertEqual(robot_ops_list[2].name, rob_op2.name)

        """ station_op list adapter """
        station_ops_list = OpListAdapter(test_model, "station_ops", StationFactory)

        # len operation
        self.assertEqual(len(station_ops_list), 2)

        # get operation
        self.assertEqual(
            station_ops_list[0].target_temperature, station_op1.target_temperature
        )

        # append operation
        station_ops_list.append(station_op3)
        self.assertEqual(len(station_ops_list), 3)
        self.assertEqual(
            station_ops_list[-1].target_duration, station_op3.target_duration
        )

        # empty list
        self.assertTrue(station_ops_list)
        station_ops_list.pop()
        station_ops_list.pop()
        station_ops_list.pop()
        self.assertEqual(len(station_ops_list), 0)
        self.assertFalse(station_ops_list)
        station_ops_list.append(station_op1)
        self.assertEqual(len(station_ops_list), 1)
        self.assertTrue(station_ops_list)

        """ state objects list adapter """
        batches_list_adapter = StateObjListAdapter(test_model, "batches", Batch)

        # len operation
        self.assertEqual(len(batches_list_adapter), 2)

        # get operation
        self.assertEqual(batches_list_adapter[0].id, batch1.id)

        # set operation
        batches_list_adapter[0] = batch3
        self.assertEqual(batches_list_adapter[0].id, batch3.id)
        batches_list_adapter[0] = batch1

        # append operation
        batches_list_adapter.append(batch3)
        self.assertEqual(len(batches_list_adapter), 3)
        self.assertEqual(batches_list_adapter[-1].id, batch3.id)

        # pop operation
        rpop_batch = batches_list_adapter.pop()
        self.assertEqual(len(batches_list_adapter), 2)
        self.assertEqual(rpop_batch.id, batch3.id)
        lpop_batch = batches_list_adapter.popleft()
        self.assertEqual(len(batches_list_adapter), 1)
        self.assertEqual(lpop_batch.id, batch1.id)

        # extend operation
        batches_list_adapter.extend([batch1, batch3])
        self.assertEqual(len(batches_list_adapter), 3)
        self.assertEqual(batches_list_adapter[0].id, batch2.id)
        self.assertEqual(batches_list_adapter[1].id, batch1.id)
        self.assertEqual(batches_list_adapter[2].id, batch3.id)

        # iteration operation
        i = 0
        ids = [2, 1, 3]
        for batch in batches_list_adapter:
            self.assertEqual(batch.id, ids[i])
            i += 1

        # remove operation
        batches_list_adapter.remove(batch1)
        self.assertEqual(len(batches_list_adapter), 2)
        self.assertEqual(batches_list_adapter[0].id, batch2.id)
        self.assertEqual(batches_list_adapter[1].id, batch3.id)


if __name__ == "__main__":
    connect(
        db="archemist_test", host="mongodb://localhost:27017", alias="archemist_state"
    )
    unittest.main()
