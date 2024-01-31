#!/usr/bin/env python3

import unittest

from mongoengine import connect
from bson.objectid import ObjectId
from time import sleep

from archemist.stations.apc_fumehood_station.state import (APCFumehoodStation,
                                                           APCCartridge,
                                                           APCDispenseSolidOp,
                                                           APCOpenSashOp,
                                                           APCCloseSashOp)
from archemist.stations.apc_fumehood_station.process import APCSolidAdditionProcess
from archemist.stations.apc_fumehood_station.handler import SimAPCFumehoodStationHandler
from archemist.core.state.robot_op import RobotTaskOp, RobotWaitOp
from archemist.core.util.enums import StationState
from archemist.core.state.station_op_result import MaterialOpResult
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.util.enums import OpOutcome, ProcessStatus
from datetime import date
from testing_utils import test_req_robot_ops, test_req_station_op, test_req_station_proc

class MTSynthesisStationTest(unittest.TestCase):
    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        self.station_doc = {
            'type': 'APCFumehoodStation',
            'id': 21,
            'location': {'coordinates': [1,7], 'descriptor': "APCFumehoodStation"},
            'total_lot_capacity': 1,
            'handler': 'SimStationOpHandler',
            'properties': {
                'cartridges': [
                    {'associated_solid': "NaCl", 'hotel_index': 1},
                    {'associated_solid': "NaCl", 'hotel_index': 2}
                    ]
            },
            'materials':
            {
                'solids': [{
                    'name': 'NaCl',
                    'amount': 100,
                    'unit': 'mg',
                    'expiry_date': date.fromisoformat('2025-02-11'),
                    'details': None
                }]
            }
        }

    def  tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_cartrdige(self):
        cartridge_dict = {'associated_solid': "NaCl", 'hotel_index': 1}
        cartridge = APCCartridge.from_dict(cartridge_dict)
        self.assertIsNotNone(cartridge)
        self.assertEqual(cartridge.associated_solid, "NaCl")
        self.assertEqual(cartridge.hotel_index, 1)
        self.assertFalse(cartridge.depleted)
        cartridge.depleted = True
        self.assertTrue(cartridge.depleted)

    def test_station_state(self):
        
        station = APCFumehoodStation.from_dict(self.station_doc)
        # test station is constructed properly
        self.assertIsNotNone(station)
        self.assertEqual(station.state, StationState.INACTIVE)

        # test station specific methods
        self.assertEqual(len(station.cartridges), 2)
        self.assertEqual(station.cartridges[0].associated_solid, "NaCl")

        self.assertIsNone(station.loaded_cartridge)
        self.assertFalse(station.sash_open)
        self.assertFalse(station.slide_window_open)
        station.slide_window_open = True
        self.assertTrue(station.slide_window_open)

        # construct lot and add it to station
        batch = Batch.from_args(1)
        lot = Lot.from_args([batch])
        station.add_lot(lot)

        # test load cartridge
        station.load_cartridge(0)
        self.assertEqual(station.loaded_cartridge.hotel_index, 1)
        self.assertFalse(station.loaded_cartridge.depleted)

        # test APCDispenseSolidOp
        t_op = APCDispenseSolidOp.from_args(target_sample=batch.samples[0],
                                                solid_name="NaCl",
                                                dispense_mass=15,
                                                dispense_unit="mg")
        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertEqual(station.solids_dict["NaCl"].mass, 85)
        self.assertTrue(station.loaded_cartridge.depleted)

        # test unload_cartridge 
        station.unload_cartridge()
        self.assertIsNone(station.loaded_cartridge)

        # test APCOpenSashop
        t_op = APCOpenSashOp.from_args()
        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertTrue(station.sash_open)

        # test APCCloseSashop
        t_op = APCCloseSashOp.from_args()
        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertFalse(station.sash_open)

    def test_process(self):

        # construct station
        station = APCFumehoodStation.from_dict(self.station_doc)
        
        batch = Batch.from_args(1)
        lot = Lot.from_args([batch])
        
        # add batches to station
        station.add_lot(lot)

        # create station process
        operations = [
                {
                    "name": "add_solid",
                    "op": "APCDispenseSolidOp",
                    "parameters": {
                        "solid_name": "NaCl",
                        "dispense_mass": 15,
                        "dispense_unit": "mg"                    
                    }
                }]

        process = APCSolidAdditionProcess.from_args(lot=lot,
                                              target_batch_index=0,
                                              target_sample_index=0,
                                              operations=operations)
        process.lot_slot = 0
        process.assigned_to = station.object_id
        # assert initial state
        self.assertEqual(process.m_state, 'init_state')
        self.assertEqual(process.status, ProcessStatus.INACTIVE)

        # prep_state
        process.tick()
        self.assertEqual(process.status, ProcessStatus.RUNNING)
        self.assertEqual(process.m_state, 'prep_state')

        # open_slide_window
        process.tick()
        self.assertEqual(process.m_state, 'open_slide_window')
        test_req_robot_ops(self, process, [RobotTaskOp, RobotWaitOp])

        # update_open_slide_window
        process.tick()
        self.assertEqual(process.m_state, 'update_open_slide_window')
        self.assertTrue(station.slide_window_open)

        # load_solid_cartridge
        process.tick()
        self.assertEqual(process.m_state, 'load_solid_cartridge')
        test_req_robot_ops(self, process, [RobotTaskOp, RobotWaitOp])

        # update_loading_cartridge
        process.tick()
        self.assertEqual(process.m_state, 'update_loading_cartridge')
        self.assertIsNotNone(station.loaded_cartridge)

        # operate_solid_cartridge
        process.tick()
        self.assertEqual(process.m_state, 'operate_solid_cartridge')
        test_req_robot_ops(self, process, [RobotTaskOp, RobotWaitOp])

        # add_solid
        process.tick()
        self.assertEqual(process.m_state, 'add_solid')
        test_req_station_op(self, process, APCDispenseSolidOp)

        # unload_solid_cartridge
        process.tick()
        self.assertEqual(process.m_state, 'unload_solid_cartridge')
        test_req_robot_ops(self, process, [RobotTaskOp, RobotWaitOp])

        # update_unloading_cartridge
        process.tick()
        self.assertEqual(process.m_state, 'update_unloading_cartridge')
        self.assertIsNone(station.loaded_cartridge)

        # close_slide_window
        process.tick()
        self.assertEqual(process.m_state, 'close_slide_window')
        test_req_robot_ops(self, process, [RobotTaskOp])

        # update_close_slide_window
        process.tick()
        self.assertEqual(process.m_state, 'update_close_slide_window')
        self.assertFalse(station.slide_window_open)

        # final_state
        process.tick()
        self.assertEqual(process.m_state, 'final_state')

    def test_sim_handler(self):
        # construct station
        station = APCFumehoodStation.from_dict(self.station_doc)

        batch_1 = Batch.from_args(1)
        lot = Lot.from_args([batch_1])

        # add batches to station
        station.add_lot(lot)

        # construct handler
        handler = SimAPCFumehoodStationHandler(station)

        # initialise the handler
        self.assertTrue(handler.initialise())

        # construct solid dispense op
        station.load_cartridge(0)
        t_op = APCDispenseSolidOp.from_args(target_sample=lot.batches[0].samples[0],
                                            solid_name='NaCl',
                                            dispense_mass=10,
                                            dispense_unit="mg")
        station.add_station_op(t_op)
        station.update_assigned_op()
        
        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(op_results), 1)
        self.assertEqual(op_results[0].origin_op, t_op.object_id)
        self.assertTrue(isinstance(op_results[0], MaterialOpResult))
        self.assertEqual(op_results[0].material_names[0], "NaCl")
        self.assertEqual(op_results[0].amounts[0], 10)
        self.assertEqual(op_results[0].units[0], "mg")

        station.complete_assigned_op(outcome, op_results)



if __name__ == '__main__':
    unittest.main()