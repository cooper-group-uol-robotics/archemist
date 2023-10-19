import unittest
from mongoengine import connect
from pathlib import Path
from time import sleep, time

from archemist.core.state.state import InputState, WorkflowState, OutputState
from archemist.core.processing.scheduler import PriorityQueueRobotScheduler
from archemist.core.processing.workflow_manager import WorkflowManager
from archemist.core.state.robot import FixedRobot
from archemist.core.state.station import Station
from archemist.core.processing.handler import StationProcessHandler
from archemist.core.util.enums import LotStatus


class WorkflowManagerTest(unittest.TestCase):
    
    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        if self._db_name in self._client.list_database_names():
            self.tearDown()

        self._recipes_path = Path.joinpath(Path.cwd(), "tests/processing/resources")

        self.input_dict = {
            "location":  {'node_id': 1, 'graph_id': 7},
            "samples_per_batch": 3,
            "batches_per_lot": 1,
            "total_lot_capacity": 2,
            "lot_input_process": {
                "type": "StationProcess",
                "args": None
            }
        }

        self.output_dict = {
            "location":  {'node_id': 12, 'graph_id': 7},
            "total_lot_capacity": 2,
            "lot_output_process": None,
            "lots_need_manual_removal": True
        }

        fixed_robot_dict = {
            "type": "FixedRobot",
            "id": 187,
            "handler": "SimRobotOpHandler"
        }

        self.fixed_robot = FixedRobot.from_dict(fixed_robot_dict)

        station_1_dict = {
            'type': 'Station',
            'id': 1,
            'location': {'node_id': 1, 'graph_id': 7},
            'handler': 'SimStationOpHandler',
            'total_lot_capacity': 2
        }
        self.station_1 = Station.from_dict(station_1_dict)

        station_2_dict = {
            'type': 'Station',
            'id': 2,
            'location': {'node_id': 2, 'graph_id': 7},
            'handler': 'SimStationOpHandler',
            'total_lot_capacity': 2
        }
        self.station_2 = Station.from_dict(station_2_dict)

    def tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_workflow_manager_process_loop(self):
        input_state = InputState.from_dict(self.input_dict)
        workflow_state = WorkflowState.from_args("test_workflow")
        output_state = OutputState.from_dict(self.output_dict)
        robot_scheduler = PriorityQueueRobotScheduler()

        workflow_manager = WorkflowManager(input_state, workflow_state,
                                           output_state, robot_scheduler,
                                           self._recipes_path)
        
        station_1_proc_handler = StationProcessHandler(self.station_1)
        station_2_proc_handler = StationProcessHandler(self.station_2)
       
        # test start workflow
        self.assertFalse(input_state.recipes_queue)
        workflow_manager.start_workflow()
        sleep(2)
        self.assertEqual(len(input_state.recipes_queue), 2)

        # test add a clean batch
        self.assertEqual(input_state.get_lots_num(), 0)
        workflow_manager.add_clean_batch()
        sleep(1)
        self.assertEqual(input_state.get_lots_num(), 1)
        workflow_manager.add_clean_batch()
        sleep(1)
        self.assertEqual(input_state.get_lots_num(), 2)
        
        # test finishing input processes
        t_start = time()
        timeout = 2

        while input_state.get_lots_num() > 0 or (time() - t_start) < timeout:
            sleep(0.5)
        self.assertEqual(input_state.get_lots_num(), 0)
        self.assertEqual(len(input_state.procs_history), 2)

        # test processing lots in station 1
        # tick to start processing lots
        station_1_proc_handler.handle()
        # tick to update procs
        station_1_proc_handler.handle()
        # tick to remove processed lots
        station_1_proc_handler.handle()
        sleep(2)

        # test processing lots in station 2
        # tick to start processing lots
        station_2_proc_handler.handle()
        # tick to update procs
        station_2_proc_handler.handle()
        # tick to remove processed lots
        station_2_proc_handler.handle()
        sleep(3)

        # test finishing output processes
        t_start = time()
        timeout = 3

        while (time() - t_start) < timeout:
            if output_state.get_lots_num() == 0:
                break
            sleep(0.5)
        self.assertEqual(output_state.get_lots_num(LotStatus.NEED_REMOVAL), 2)

        # test removing lot
        workflow_manager.remove_lot(0)
        sleep(1)
        self.assertEqual(output_state.get_lots_num(LotStatus.NEED_REMOVAL), 1)

        # test removing all lots
        workflow_manager.remove_all_lots()
        sleep(1)
        self.assertEqual(output_state.get_lots_num(), 0)
        self.assertEqual(len(output_state.procs_history), 2)

        # test that all the stations have been visited
        self.assertEqual(len(self.station_1.procs_history), 2)
        self.assertEqual(self.station_1.free_lot_capacity, 2)
        
        self.assertEqual(len(self.station_2.procs_history), 2)
        self.assertEqual(self.station_2.free_lot_capacity, 2)

        workflow_manager.stop_processor()
        sleep(2)
        

if __name__ == "__main__":
    unittest.main()
        
