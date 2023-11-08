import unittest
from mongoengine import connect

from archemist.stations.quantos_qs2_station.state import (QuantosSolidDispenserQS2,
                                                          QuantosCartridge,
                                                          QuantosOpenDoorOp,
                                                          QuantosCloseDoorOp,
                                                          QuantosLoadCartridgeOp,
                                                          QuantosUnloadCartridgeOp,
                                                          QuantosMoveCarouselOp,
                                                          QuantosDispenseOp)
from archemist.stations.quantos_qs2_station.handler import SimQuantosSolidDispenserQS2Handler
from archemist.core.state.station_op_result import MaterialOpResult
from archemist.core.state.lot import Lot, Batch
from archemist.core.util.enums import StationState, OpOutcome
from datetime import date

class QuantosSolidDispenserQS2Test(unittest.TestCase):
    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        station_dict = {
            'type': 'QuantosSolidDispenserQS2',
            'id': 20,
            'location': {'coordinates': [1,7], 'descriptor': "ChemSpeedFlexStation"},
            'total_lot_capacity': 1,
            'handler': 'SimStationOpHandler',
            'properties': {
                'cartridges': [{'associated_solid': "NaCl", 'hotel_index': 1, 'remaining_dosages': 100}]
            },
            'materials': {
                'solids': [{
                    'name': 'NaCl',
                    'amount': 100,
                    'unit': 'mg',
                    'expiry_date': date.fromisoformat('2025-02-11'),
                    'details': None
                }]
            }
        }
        
        self.station = QuantosSolidDispenserQS2.from_dict(station_dict=station_dict)
        
    def tearDown(self):
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_quantos_cartrdige(self):
        cartridge_dict = {'associated_solid': "NaCl", 'hotel_index': 1, 'remaining_dosages': 1}
        cartridge = QuantosCartridge.from_dict(cartridge_dict)
        self.assertIsNotNone(cartridge)
        self.assertEqual(cartridge.associated_solid, "NaCl")
        self.assertEqual(cartridge.hotel_index, 1)
        self.assertEqual(cartridge.remaining_dosages, 1)
        self.assertFalse(cartridge.is_consumed())
        self.assertFalse(cartridge.blocked)
        cartridge.remaining_dosages -= 1
        self.assertEqual(cartridge.remaining_dosages, 0)
        self.assertTrue(cartridge.is_consumed())
    
    def test_state(self):
        # test station is constructed properly
        self.assertIsNotNone(self.station)
        self.assertEqual(self.station.state, StationState.INACTIVE)
        
        # test station specific methods
        self.assertEqual(len(self.station.solids_dict), 1)
        self.assertEqual(self.station.solids_dict["NaCl"].mass, 100)
        self.assertEqual(self.station.carousel_pos,1)
        self.station.carousel_pos = 2
        self.assertEqual(self.station.carousel_pos,2)
        self.assertFalse(self.station.door_open)
        self.assertEqual(len(self.station.cartridges), 1)
        self.assertEqual(self.station.cartridges[0].associated_solid, "NaCl")
        self.assertIsNone(self.station.loaded_cartridge)

        # construct lot and add it to station
        batch_1 = Batch.from_args(2)
        lot = Lot.from_args([batch_1])
        self.station.add_lot(lot)

        # test QuantosOpenDoorOp
        t_op = QuantosOpenDoorOp.from_args()
        self.assertIsNotNone(t_op)
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertTrue(self.station.door_open)

        # test QuantosLoadCartridgeOp
        t_op = QuantosLoadCartridgeOp.from_args("NaCl")
        self.assertIsNotNone(t_op)
        self.assertEqual(t_op.solid_name, "NaCl")
        self.station.add_station_op(t_op)
        self.assertEqual(len(self.station._queued_ops), 1)
        self.station.update_assigned_op()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertIsNotNone(self.station.loaded_cartridge)
        self.assertEqual(self.station.loaded_cartridge.associated_solid, "NaCl")

        # test unsueccessful QuantosLoadCartridgeOp
        t_op = QuantosLoadCartridgeOp.from_args("NaCl")
        self.assertIsNotNone(t_op)
        self.station.add_station_op(t_op)
        self.assertEqual(len(self.station._queued_ops), 0)

        # test QuantosCloseDoorOp
        t_op = QuantosCloseDoorOp.from_args()
        self.assertIsNotNone(t_op)
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertFalse(self.station.door_open)

        # test QuantosMoveCarouselOp
        t_op = QuantosMoveCarouselOp.from_args(target_pos=5)
        self.assertIsNotNone(t_op)
        self.assertEqual(t_op.target_pos, 5)
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertIsNotNone(self.station.carousel_pos, 5)

        # test QuantosDispenseOp
        t_op = QuantosDispenseOp.from_args(target_sample=batch_1.samples[0],
                                           solid_name="NaCl",
                                           dispense_mass=10,
                                           dispense_unit="mg")
        self.assertIsNotNone(t_op)
        self.assertEqual(t_op.solid_name, "NaCl")
        self.assertEqual(t_op.dispense_mass, 10)
        self.assertEqual(t_op.dispense_unit, "mg")
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()

        result = MaterialOpResult.from_args(origin_op=t_op.object_id,
                                            material_names=["NaCl"],
                                            amounts=[10],
                                            units=["mg"])

        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, [result])
        self.assertEqual(self.station.solids_dict["NaCl"].mass, 90)
        self.assertEqual(self.station.loaded_cartridge.remaining_dosages, 99)

        # test QuantosUnloadCartridgeOp
        t_op = QuantosUnloadCartridgeOp.from_args()
        self.assertIsNotNone(t_op)
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertIsNone(self.station.loaded_cartridge)

    def test_sim_handler(self):
        batch_1 = Batch.from_args(3)
        lot = Lot.from_args([batch_1])

        # add batches to station
        self.station.add_lot(lot)

        # construct handler
        handler = SimQuantosSolidDispenserQS2Handler(self.station)

        # initialise the handler
        self.assertTrue(handler.initialise())

        # load cartidge
        t_op = QuantosLoadCartridgeOp.from_args("NaCl")
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertIsNone(op_results)
        self.station.complete_assigned_op(outcome, op_results)

        # dispense solid
        t_op = QuantosDispenseOp.from_args(target_sample=batch_1.samples[0],
                                           solid_name="NaCl",
                                           dispense_mass=10,
                                           dispense_unit="mg")
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        
        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(op_results), 1)
        self.assertEqual(op_results[0].origin_op, t_op.object_id)
        self.assertTrue(isinstance(op_results[0], MaterialOpResult))
        self.assertEqual(op_results[0].material_names[0], "NaCl")
        self.assertEqual(op_results[0].amounts[0], 10)
        self.assertEqual(op_results[0].units[0], "mg")

        self.station.complete_assigned_op(outcome, op_results)

if __name__ == '__main__':
    unittest.main()