import unittest

from mongoengine import connect

from archemist.stations.mt_synthesis_station.state import (MTSynthesisStation,
                                                           MTSynthSampleOp,
                                                           MTSynthStopReactionOp,
                                                           MTSynthOpenReactionValveOp,
                                                           MTSynthCloseReactionValveOp,
                                                           MTSynthTimedOpenReactionValveOp,
                                                           OptiMaxMode,
                                                           MTSynthHeatStirOp)
from archemist.stations.mt_synthesis_station.handler import SimMTSynthesisStationHandler
from archemist.core.util.enums import StationState
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.util.enums import OpOutcome


class MTSynthesisStationTest(unittest.TestCase):
    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        self.station_doc = {
            'type': 'MTSynthesisStation',
            'id': 21,
            'location': {'coordinates': [1, 7], 'descriptor': "MTSynthesisStation"},
            'total_lot_capacity': 1,
            'handler': 'SimStationOpHandler',
            'properties':
            {
                'num_sampling_vials': 12
            },
            'materials': None
        }

    def tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_station_state(self):

        station = MTSynthesisStation.from_dict(self.station_doc)
        # test station is constructed properly
        self.assertIsNotNone(station)
        self.assertEqual(station.state, StationState.INACTIVE)

        # test station specific methods
        self.assertIsNone(station.optimax_mode)
        self.assertFalse(station.optimax_valve_open)
        self.assertEqual(station.num_sampling_vials, 12)
        self.assertIsNone(station.set_reaction_temperature)
        self.assertIsNone(station.set_stirring_speed)

        # construct lot and add it to station
        batch = Batch.from_args(1)
        lot = Lot.from_args([batch])
        station.add_lot(lot)

        # test MTSynthHeatStirOp heating and stirring
        t_op = MTSynthHeatStirOp.from_args(target_sample=batch.samples[0],
                                           target_temperature=100,
                                           target_stirring_speed=50,
                                           wait_duration=3,
                                           time_unit="minute")
        self.assertIsNotNone(t_op)
        self.assertEqual(t_op.target_temperature, 100)
        self.assertEqual(t_op.target_stirring_speed, 50)
        self.assertEqual(t_op.wait_duration, 3)
        self.assertEqual(t_op.time_unit, "minute")

        station.add_station_op(t_op)
        station.update_assigned_op()
        self.assertEqual(station.optimax_mode, OptiMaxMode.HEATING_STIRRING)
        self.assertEqual(station.set_reaction_temperature, 100)
        self.assertEqual(station.set_stirring_speed, 50)
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertIsNone(station.optimax_mode)
        self.assertEqual(station.set_reaction_temperature, 100)
        self.assertEqual(station.set_stirring_speed, 50)

        # test MTSynthHeatStirOp heating
        t_op = MTSynthHeatStirOp.from_args(target_sample=batch.samples[0],
                                           target_temperature=103,
                                           target_stirring_speed=None,
                                           wait_duration=None,
                                           time_unit=None)
        self.assertIsNotNone(t_op)
        self.assertEqual(t_op.target_temperature, 103)
        self.assertIsNone(t_op.target_stirring_speed)
        self.assertIsNone(t_op.wait_duration)
        self.assertIsNone(t_op.time_unit)

        station.add_station_op(t_op)
        station.update_assigned_op()
        self.assertEqual(station.optimax_mode, OptiMaxMode.HEATING)
        self.assertEqual(station.set_reaction_temperature, 103)
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertIsNone(station.optimax_mode)

        # test MTSynthHeatStirOp stirring
        t_op = MTSynthHeatStirOp.from_args(target_sample=batch.samples[0],
                                           target_temperature=None,
                                           target_stirring_speed=55,
                                           wait_duration=None,
                                           time_unit=None)
        self.assertIsNotNone(t_op)
        self.assertEqual(t_op.target_stirring_speed, 55)
        self.assertIsNone(t_op.target_temperature)
        self.assertIsNone(t_op.wait_duration)
        self.assertIsNone(t_op.time_unit)

        station.add_station_op(t_op)
        station.update_assigned_op()
        self.assertEqual(station.optimax_mode, OptiMaxMode.STIRRING)
        self.assertEqual(station.set_stirring_speed, 55)
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertIsNone(station.optimax_mode)

        # test MTSynthSampleOp heating
        t_op = MTSynthSampleOp.from_args(target_sample=batch.samples[0],
                                         target_temperature=99,
                                         target_stirring_speed=None)
        self.assertIsNotNone(t_op)
        self.assertEqual(t_op.target_temperature, 99)
        self.assertIsNone(t_op.target_stirring_speed)

        station.add_station_op(t_op)
        station.update_assigned_op()
        self.assertEqual(station.optimax_mode, OptiMaxMode.HEATING)
        self.assertEqual(station.set_reaction_temperature, 99)
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertEqual(station.num_sampling_vials, 11)
        self.assertEqual(station.optimax_mode, OptiMaxMode.HEATING)

        # test MTSynthStopReactionOp
        t_op = MTSynthStopReactionOp.from_args()
        self.assertIsNotNone(t_op)

        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertIsNone(station.optimax_mode)

        # test MTSynthOpenReactionValveOp
        t_op = MTSynthOpenReactionValveOp.from_args()
        self.assertIsNotNone(t_op)

        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertTrue(station.optimax_valve_open)

        # test MTSynthCloseReactionValveOp
        t_op = MTSynthCloseReactionValveOp.from_args()
        self.assertIsNotNone(t_op)

        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertFalse(station.optimax_valve_open)

        # test MTSynthTimedOpenReactionValveOp
        t_op = MTSynthTimedOpenReactionValveOp.from_args(duration=1.5,
                                                         time_unit="second")
        self.assertIsNotNone(t_op)

        station.add_station_op(t_op)
        station.update_assigned_op()
        self.assertTrue(station.optimax_valve_open)
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertFalse(station.optimax_valve_open)

    def test_sim_handler(self):
        # construct station
        station = MTSynthesisStation.from_dict(self.station_doc)

        batch_1 = Batch.from_args(1)
        lot = Lot.from_args([batch_1])

        # add batches to station
        station.add_lot(lot)

        # construct handler
        handler = SimMTSynthesisStationHandler(station)

        # initialise the handler
        self.assertTrue(handler.initialise())

        # construct MTSynthHeatStirOp
        t_op = MTSynthHeatStirOp.from_args(target_sample=lot.batches[0].samples[0],
                                           target_temperature=100,
                                           target_stirring_speed=110,
                                           wait_duration=3,
                                           time_unit="minute")
        station.add_station_op(t_op)
        station.update_assigned_op()

        # get op
        parameters = {}
        parameters["target_temperature"] = t_op.target_temperature
        parameters["target_stirring_speed"] = t_op.target_stirring_speed
        parameters["wait_duration"] = t_op.wait_duration
        parameters["time_unit"] = t_op.time_unit

        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(op_results), 1)
        self.assertEqual(op_results[0].origin_op, t_op.object_id)
        self.assertDictEqual(op_results[0].parameters, parameters)

        station.complete_assigned_op(outcome, op_results)

        # construct MTSynthSampleOp
        t_op = MTSynthSampleOp.from_args(target_sample=lot.batches[0].samples[0],
                                         target_temperature=100,
                                         target_stirring_speed=110)
        station.add_station_op(t_op)
        station.update_assigned_op()

        # get op
        parameters = {}
        parameters["target_temperature"] = t_op.target_temperature
        parameters["target_stirring_speed"] = t_op.target_stirring_speed

        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(op_results), 1)
        self.assertEqual(op_results[0].origin_op, t_op.object_id)
        self.assertDictEqual(op_results[0].parameters, parameters)

        station.complete_assigned_op(outcome, op_results)
