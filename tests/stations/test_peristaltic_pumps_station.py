import unittest
from datetime import datetime

from mongoengine import connect

from archemist.stations.peristaltic_pumps_station.state import (PeristalticPumpsStation,
                                                                PPLiquidDispenseOp)
from archemist.core.util.enums import StationState, OpOutcome, ProcessStatus
from archemist.stations.peristaltic_pumps_station.handler import SimPeristalticPumpsStationHandler
from archemist.stations.peristaltic_pumps_station.process import PandaPumpSolubilityProcess
from archemist.core.state.robot_op import RobotTaskOp
from archemist.core.state.lot import Lot, Batch
from archemist.core.state.station_op_result import MaterialOpResult
from .testing_utils import test_req_robot_ops, test_req_station_op


class PeristalticLiquidDispensingTest(unittest.TestCase):
    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        station_dict = {
            'type': 'PeristalticPumpsStation',
            'id': 20,
            'location': {'coordinates': [1, 7], 'descriptor': "ChemSpeedFlexStation"},
            'total_lot_capacity': 1,
            'handler': 'SimStationOpHandler',
            'materials':
            {
                'liquids':
                [{
                    'name': 'water',
                    'amount': 400,
                    'unit': 'mL',
                    'density': 997,
                    'density_unit': "kg/m3",
                    "details": None,
                    'expiry_date': datetime.fromisoformat('2025-02-11')
                }]
            },
            'properties': {
                'liquid_pump_map': {'water': 1}
            }
        }

        self.station = PeristalticPumpsStation.from_dict(station_dict)

    def tearDown(self):
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_state(self):
        # test station is constructed properly
        self.assertIsNotNone(self.station)
        self.assertEqual(self.station.state, StationState.INACTIVE)

        # construct lot and add it to station
        batch_1 = Batch.from_args(2)
        lot = Lot.from_args([batch_1])
        self.station.add_lot(lot)

        # test station specific fields
        self.assertDictEqual(dict(self.station.liquid_pump_map), {'water': 1})

        # test PPLiquidDispenseOp
        t_op = PPLiquidDispenseOp.from_args(target_sample=lot.batches[0].samples[0],
                                            liquid_name='water',
                                            dispense_volume=100,
                                            dispense_unit="mL")
        self.assertIsNotNone(t_op.object_id)
        self.assertEqual(t_op.liquid_name, "water")
        self.assertEqual(t_op.dispense_volume, 100)
        self.assertEqual(t_op.dispense_unit, "mL")

        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        op_result = MaterialOpResult.from_args(t_op.object_id,
                                               [t_op.liquid_name],
                                               [t_op.dispense_volume],
                                               [t_op.dispense_unit])

        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, [op_result])
        self.assertEqual(self.station.liquids_dict["water"].volume, 300)
        self.assertEqual(self.station.liquids_dict["water"].volume_unit, "mL")

    def test_solubility_process(self):

        batch = Batch.from_args(2)
        lot = Lot.from_args([batch])

        # add batches to station
        self.station.add_lot(lot)

        # create station process
        operations = [
            {
                "name": "dispense_op",
                "op": "PPLiquidDispenseOp",
                "parameters": {
                        "liquid_name": "water",
                        "dispense_volume": 10,
                        "dispense_unit": "mL"
                }
            }
        ]
        process = PandaPumpSolubilityProcess.from_args(lot=lot,
                                                       operations=operations)
        process.lot_slot = 0

        # assert initial state
        self.assertEqual(process.m_state, 'init_state')
        self.assertEqual(process.status, ProcessStatus.INACTIVE)

        # prep_state
        process.tick()
        self.assertEqual(process.status, ProcessStatus.RUNNING)
        self.assertEqual(process.m_state, 'prep_state')

        # move_dispense_head
        process.tick()
        self.assertEqual(process.m_state, 'move_dispense_head')
        test_req_robot_ops(self, process, [RobotTaskOp])

        # dispense_liquid
        process.tick()
        self.assertEqual(process.m_state, 'dispense_liquid')
        test_req_station_op(self, process, PPLiquidDispenseOp)

        # replace_dispense_head
        process.tick()
        self.assertEqual(process.m_state, 'replace_dispense_head')
        test_req_robot_ops(self, process, [RobotTaskOp])

        # final_state
        process.tick()
        self.assertEqual(process.m_state, 'final_state')
        self.assertEqual(process.status, ProcessStatus.FINISHED)

    def test_sim_handler(self):
        batch_1 = Batch.from_args(3)
        lot = Lot.from_args([batch_1])

        # add batches to station
        self.station.add_lot(lot)

        # construct handler
        handler = SimPeristalticPumpsStationHandler(self.station)

        # initialise the handler
        self.assertTrue(handler.initialise())

        # construct analyse op
        t_op = PPLiquidDispenseOp.from_args(target_sample=lot.batches[0].samples[0],
                                            liquid_name='water',
                                            dispense_volume=100,
                                            dispense_unit="mL")
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()

        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(op_results), 1)
        self.assertEqual(op_results[0].origin_op, t_op.object_id)
        self.assertTrue(isinstance(op_results[0], MaterialOpResult))
        self.assertEqual(op_results[0].material_names[0], "water")
        self.assertEqual(op_results[0].amounts[0], 100)
        self.assertEqual(op_results[0].units[0], "mL")

        self.station.complete_assigned_op(outcome, op_results)


if __name__ == '__main__':
    unittest.main()
