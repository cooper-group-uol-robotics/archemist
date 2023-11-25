import unittest
from archemist.core.state.robot_op import RobotTaskOp
from mongoengine import connect
from archemist.core.state.station import Station, StationState, OpState, OpOutcome
from archemist.core.state.station_op import StationOp
from archemist.core.state.station_op_result import StationOpResult
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.util.location import Location
from archemist.core.util.enums import LotStatus
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
            'location': {'coordinates': [1,2], 'descriptor': "Station"},
            'handler': 'SimStationOpHandler',
            'total_lot_capacity': 2,
            'materials': {
                'liquids': [
                    {
                    'name': 'water',
                    'amount': 400,
                    'unit': 'mL',
                    'density': 997,
                    'density_unit': 'kg/m3',
                    'expiry_date': date.fromisoformat('2025-02-11'),
                    'details': {'pump_id': 'pUmP1'}
                    }],
                'solids': [
                    {
                    'name': 'sodium_chloride',
                    'amount': 10000,
                    'unit': 'ug',
                    'expiry_date': date.fromisoformat('2025-02-11'),
                    'details': {'dispense_src': 'quantos', 'cartridge_id': 123}
                    }]
            }
        }

        self.station: Station = Station.from_dict(station_dict=station_dict)

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
        self.assertEqual(self.station.location, Location.from_dict({'coordinates': [1,2], 'descriptor': "Station"}))
    
    def test_material_members(self):
        liquids = self.station.liquids_dict
        self.assertEqual(len(liquids), 1)
        self.assertEqual(liquids["water"].volume, 400)
        self.assertEqual(liquids["water"].belongs_to, self.station.object_id)

        solids = self.station.solids_dict
        self.assertEqual(len(solids), 1)
        self.assertEqual(solids["sodium_chloride"].mass, 10000)
        self.assertEqual(solids["sodium_chloride"].belongs_to, self.station.object_id)


    def test_lot_members(self):
        # assert empty members
        self.assertIsNone(self.station.lot_slots["0"])
        self.assertIsNone(self.station.lot_slots["1"])

        # lots creation
        batch_1 = Batch.from_args(3)
        lot_1 = Lot.from_args([batch_1])
        batch_2 = Batch.from_args(3)
        lot_2 = Lot.from_args([batch_2])
        

        # lot assignment
        self.assertEqual(self.station.free_lot_capacity, 2)
        self.assertFalse(self.station.is_lot_onboard(lot_1))
        self.station.add_lot(lot_1)
        self.assertEqual(self.station.free_lot_capacity, 1)
        self.assertEqual(self.station.lot_slots["0"], lot_1)
        self.assertTrue(self.station.is_lot_onboard(lot_1))
        self.station.add_lot(lot_2)
        self.assertEqual(self.station.free_lot_capacity, 0)
        self.assertEqual(self.station.lot_slots["1"], lot_2)
        self.assertTrue(self.station.is_lot_onboard(lot_2))

        # lot processing
        self.assertFalse(self.station.has_ready_for_collection_lots())
        self.station.finish_processing_lot(lot_1)
        self.assertEqual(self.station.lot_slots["0"].status, LotStatus.READY_FOR_COLLECTION)
        self.assertEqual(self.station.free_lot_capacity, 0)
        self.assertTrue(self.station.has_ready_for_collection_lots())
        collected_lots = self.station.retrieve_ready_for_collection_lots()
        self.assertEqual(len(collected_lots), 1)
        self.assertEqual(collected_lots[0], lot_1)
        self.assertEqual(self.station.free_lot_capacity, 1)
        self.assertIsNone(self.station.lot_slots["0"])

        self.station.finish_processing_lot(lot_2)
        self.assertEqual(self.station.lot_slots["1"].status, LotStatus.READY_FOR_COLLECTION)
        self.assertEqual(self.station.free_lot_capacity, 1)
        self.assertTrue(self.station.has_ready_for_collection_lots())
        collected_lots = self.station.retrieve_ready_for_collection_lots()
        self.assertEqual(len(collected_lots), 1)
        self.assertEqual(collected_lots[0], lot_2)
        self.assertEqual(self.station.free_lot_capacity, 2)
        self.assertIsNone(self.station.lot_slots["1"])

    def test_robot_ops_members(self):
        # assert empty members
        self.assertFalse(self.station.requested_robot_ops)

        # op creation
        robot_op1 = RobotTaskOp.from_args('test_task1', "Robot", params={'calibrate': False, 'index': 1})
        robot_op2 = RobotTaskOp.from_args('test_task2', "Robot", params={'calibrate': False, 'index': 1})
        
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
        station_op_1 = StationOp.from_args()
        station_op_2 = StationOp.from_args()

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
        self.assertFalse(self.station.running_procs)
        self.assertEqual(self.station.num_running_main_procs, 0)
        self.assertFalse(self.station.procs_history)

        # construct process
        batch_1 = Batch.from_args(3)
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

        # add main process to running_procs
        self.station.running_procs.append(ext_proc)
        self.assertEqual(len(self.station.running_procs), 1)
        self.assertEqual(self.station.num_running_main_procs, 1)
        self.assertEqual(self.station.running_procs[0].object_id, ext_proc.object_id)

        # add sub process to running_procs
        sub_proc = StationProcess.from_args(lot, is_subprocess=True)
        self.station.running_procs.append(sub_proc)
        self.assertEqual(len(self.station.running_procs), 2)
        self.assertEqual(self.station.num_running_main_procs, 1)
        self.assertEqual(self.station.running_procs[1].object_id, sub_proc.object_id)

if __name__ == '__main__':
    unittest.main()