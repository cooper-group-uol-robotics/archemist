import unittest
from archemist.core.state.robot import RobotTaskOpDescriptor
from mongoengine import connect
from archemist.core.state.station import Station, StationState, OpState, OpOutcome
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.state.station_op_result import StationOpResult
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.util.location import Location
from archemist.core.state.station_process import StationProcess
from archemist.core.state.material import Liquid, Solid
from datetime import date

class StationTest(unittest.TestCase):

    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')
        station_dict = {
            'type': 'Station',
            'id': 23,
            'location': {'node_id': 1, 'graph_id': 7},
            'handler': 'SimStationOpHandler',
            'total_lot_capacity': 2
        }

        liquid_dict = {
            'name': 'water',
            'id': 1254,
            'amount': 400,
            'unit': 'mL',
            'density': 997,
            'density_unit': 'kg/m3',
            'expiry_date': date.fromisoformat('2025-02-11'),
            'details': {
                'pump_id': 'pUmP1'
                }
        }
        water = Liquid.from_dict(liquid_dict)

        solid_dict = {
            'name': 'sodium_chloride',
            'id': 133,
            'amount': 10000,
            'unit': 'ug',
            'expiry_date': date.fromisoformat('2025-02-11'),
            'details': {'dispense_src': 'quantos', 'cartridge_id': 123}
        }
        salt = Solid.from_dict(solid_dict)

        self.station: Station = Station.from_dict(station_dict=station_dict, liquids=[water], solids=[salt])

    def  tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()  

    def test_general_members(self):
        self.assertEqual(self.station.id, 23)
        self.assertIsNotNone(self.station.model)
        self.assertIsNotNone(self.station.object_id)
        self.assertEqual(self.station.state, StationState.INACTIVE)
        self.station.state = StationState.ACTIVE
        self.assertEqual(self.station.state, StationState.ACTIVE)
        self.assertEqual(self.station.total_lot_capacity, 2)
        self.assertEqual(self.station.module_path, "archemist.core.state.station")
        self.assertEqual(self.station.selected_handler, "SimStationOpHandler")
        self.assertEqual(self.station.location, Location(1,7,''))
    
    def test_material_members(self):
        liquids = self.station.liquids
        self.assertEqual(len(liquids), 1)
        self.assertEqual(liquids[0].id, 1254)

        solids = self.station.solids
        self.assertEqual(len(solids), 1)
        self.assertEqual(solids[0].id, 133)

    def test_lot_members(self):
        # assert empty members
        self.assertFalse(self.station.assigned_lots)
        self.assertFalse(self.station.processed_lots)

        # lots creation
        batch_1 = Batch.from_args(3, Location(1, 2, "some_frame"))
        lot_1 = Lot.from_args([batch_1])
        batch_2 = Batch.from_args(3, Location(1, 2, "some_frame"))
        lot_2 = Lot.from_args([batch_2])
        

        # lot assignment
        self.assertEqual(self.station.free_lot_capacity, 2)
        self.station.add_lot(lot_1)
        self.assertEqual(self.station.free_lot_capacity, 1)
        self.station.add_lot(lot_2)
        self.assertEqual(self.station.free_lot_capacity, 0)

        assigned_lots = self.station.assigned_lots
        self.assertEqual(len(assigned_lots),2)
        self.assertEqual(assigned_lots[0], lot_1)
        self.assertEqual(assigned_lots[1], lot_2)

        # lot processing
        self.station.finish_processing_lot(lot_1)
        self.assertEqual(len(self.station.assigned_lots), 1)
        self.assertEqual(len(self.station.processed_lots), 1)
        self.assertEqual(self.station.processed_lots[0], lot_1)
        self.assertEqual(self.station.free_lot_capacity, 0)

        self.station.finish_processing_lot(lot_2)
        self.assertEqual(len(self.station.assigned_lots), 0)
        self.assertEqual(len(self.station.processed_lots), 2)
        self.assertEqual(self.station.processed_lots[1], lot_2)
        self.assertEqual(self.station.free_lot_capacity, 0)

        finisehd_lot = self.station.processed_lots.pop(left=True)
        self.assertEqual(finisehd_lot, lot_1)
        self.assertEqual(self.station.free_lot_capacity, 1)

    def test_robot_ops_members(self):
        # assert empty members
        self.assertFalse(self.station.requested_robot_ops)

        # op creation
        robot_op1 = RobotTaskOpDescriptor.from_args('test_task1', "Robot", params={'calibrate': False, 'index': 1})
        robot_op2 = RobotTaskOpDescriptor.from_args('test_task2', "Robot", params={'calibrate': False, 'index': 1})
        
        # op assignment
        self.station.add_req_robot_op(robot_op1)
        self.assertEqual(robot_op1.requested_by, self.station.object_id)
        self.station.add_req_robot_op(robot_op2)
        self.assertEqual(robot_op2.requested_by, self.station.object_id)
        self.assertEqual(len(self.station.requested_robot_ops), 2)
        
        # op retrival
        robot_op = self.station.requested_robot_ops.pop()
        self.assertEqual(len(self.station.requested_robot_ops), 1)
        self.assertEqual(robot_op, robot_op1)

        robot_op = self.station.requested_robot_ops.pop()
        self.assertEqual(len(self.station.requested_robot_ops), 0)
        self.assertEqual(robot_op, robot_op2)
        
    def test_station_ops_members(self):
        # assert empty members
        self.assertFalse(self.station._queued_ops)
        self.assertIsNone(self.station.assigned_op)
        self.assertEqual(self.station.assigned_op_state, OpState.INVALID)
        self.assertFalse(self.station.ops_history)

        # op creation
        station_op_1 = StationOpDescriptor.from_args()
        station_op_2 = StationOpDescriptor.from_args()

        # op assignment
        self.station.add_station_op(station_op_1)
        self.station.add_station_op(station_op_2)
        
        self.assertEqual(len(self.station._queued_ops), 2)
        self.assertIsNone(self.station.assigned_op)
        self.assertEqual(self.station.assigned_op_state, OpState.INVALID)

        # process station op 1
        self.station.update_assigned_op()
        self.assertEqual(len(self.station._queued_ops), 1)
        self.assertIsNotNone(self.station.assigned_op)
        self.assertEqual(self.station.assigned_op_state, OpState.ASSIGNED)

        self.assertEqual(self.station.assigned_op, station_op_1)
        self.assertIsNone(self.station.assigned_op.start_timestamp)
        
        self.station.set_assigned_op_to_execute()
        self.assertEqual(self.station.assigned_op_state, OpState.EXECUTING)
        
        # complete station op1
        self.station.assigned_op.add_start_timestamp()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertIsNone(self.station.assigned_op)
        self.assertEqual(self.station.assigned_op_state, OpState.INVALID)
        self.assertEqual(len(self.station.ops_history), 1)
        
        complete_op = self.station.ops_history[0]
        self.assertEqual(complete_op, station_op_1)
        self.assertIsNotNone(complete_op.start_timestamp)
        self.assertEqual(complete_op.outcome, OpOutcome.SUCCEEDED)

        # process station op 2

        self.station.update_assigned_op()
        self.assertEqual(len(self.station._queued_ops), 0)
        self.assertIsNotNone(self.station.assigned_op)
        self.assertEqual(self.station.assigned_op, station_op_2)
        self.assertEqual(self.station.assigned_op_state, OpState.ASSIGNED)

        # test repeat and skip op
        self.station.repeat_assigned_op()
        self.assertEqual(self.station.assigned_op_state, OpState.TO_BE_REPEATED)

        self.station.skip_assigned_op()
        self.assertEqual(self.station.assigned_op_state, OpState.TO_BE_SKIPPED)

        # complete station op 2
        self.station.assigned_op.add_start_timestamp()
        result_op = StationOpResult.from_args(origin_op=self.station.assigned_op.object_id)
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, [result_op])
        self.assertIsNone(self.station.assigned_op)
        self.assertEqual(self.station.assigned_op_state, OpState.INVALID)
        self.assertEqual(len(self.station.ops_history), 2)
        complete_op = self.station.ops_history[1]
        self.assertEqual(complete_op, station_op_2)
        self.assertIsNotNone(complete_op.start_timestamp)
        self.assertEqual(complete_op.outcome, OpOutcome.SUCCEEDED)
        
    def test_process_members(self):
        # assert empty members
        self.assertFalse(self.station.requested_ext_procs)
        self.assertFalse(self.station.queued_procs)
        self.assertEqual(len(self.station.running_procs_slots), 2)
        self.assertIsNone(self.station.running_procs_slots["0"])
        self.assertIsNone(self.station.running_procs_slots["1"])
        self.assertFalse(self.station.procs_history)

        # construct process
        batch_1 = Batch.from_args(3, Location(1, 2, "some_frame"))
        lot = Lot.from_args([batch_1])
        proc = StationProcess.from_args(lot)

        # exteranl procs
        self.station.request_external_process(proc)
        self.assertEqual(len(self.station.requested_ext_procs),1)
        ext_proc = self.station.requested_ext_procs.pop()
        self.assertEqual(len(self.station.requested_ext_procs),0)
        self.assertEqual(ext_proc.object_id, proc.object_id)

        # normal procs
        self.station.add_process(ext_proc)
        self.assertEqual(len(self.station.queued_procs),1)
        self.assertEqual(self.station.queued_procs[0].object_id, proc.object_id)

        # add process to running_procs_slots
        self.station.running_procs_slots["0"] = ext_proc
        self.assertEqual(self.station.running_procs_slots["0"].object_id, ext_proc.object_id)

if __name__ == '__main__':
    unittest.main()