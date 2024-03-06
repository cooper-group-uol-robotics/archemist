import unittest
from pathlib import Path
import zmq
from threading import Thread
from time import sleep, time
from typing import Callable, Any
import json

from archemist.application.archemist_server import ArchemistServer
from archemist.core.processing.handler import StationProcessHandler
from archemist.core.util.enums import WorkflowManagerStatus, LotStatus
from archemist.core.persistence.objects_getter import (BatchesGetter,
                                                       RecipesGetter,
                                                       LotsGetter,
                                                       StationsGetter,
                                                       StateGetter)
from archemist.application.cmd_message import CMDCategory, CMDMessage


class ArchemistServerTest(unittest.TestCase):
    def setUp(self):
        self.server = None
        self.server_thread = None
        self.client = None

    def tearDown(self) -> None:
        if self.client:
            self.client.close()
        if self.server:
            self.server.shut_down()
            self.server._persistence_mgr._db_handler.delete_database()
            self.server_thread.join(2)

    def waitTillAssertion(self, condition_eval: Callable[[Any], bool], timeout: int = 10):
        t_start = time()
        while (time() - t_start) < timeout:
            if condition_eval():
                self.assertTrue(True)
                return
            sleep(0.1)
        self.assertTrue(False)

    def test_server(self):
        # construct server
        workflow_dir = Path.joinpath(Path.cwd(), "tests/application/resources/test_workflow")
        self.server = ArchemistServer(workflow_dir, existing_db=False)

        # construct server thread and start it
        self.server_thread = Thread(target=self.server.run, daemon=True)
        self.server_thread.start()

        # construct client
        context = zmq.Context()
        self.client = context.socket(zmq.PAIR)
        self.client.connect('tcp://127.0.0.1:5555')

        # test start workflow
        msg = CMDMessage(category=CMDCategory.WORKFLOW, cmd='start')
        self.client.send_json(msg.to_json())
        sleep(0.5)

        self.assertEqual(self.server._workflow_mgr.status, WorkflowManagerStatus.RUNNING)
        self.assertEqual(len(RecipesGetter.get_recipes()), 1)

        # test pause workflow
        msg = CMDMessage(category=CMDCategory.WORKFLOW, cmd='pause')
        self.client.send_json(msg.to_json())
        sleep(0.5)
        self.assertEqual(self.server._workflow_mgr.status, WorkflowManagerStatus.PAUSED)

        # test resume workflow
        msg = CMDMessage(category=CMDCategory.WORKFLOW, cmd='start')
        self.client.send_json(msg.to_json())
        sleep(0.5)
        self.assertEqual(self.server._workflow_mgr.status, WorkflowManagerStatus.RUNNING)

        # test adding clean batch
        msg = CMDMessage(category=CMDCategory.WORKFLOW, cmd='add_batch')
        self.client.send_json(msg.to_json())
        sleep(0.5)

        self.assertEqual(len(BatchesGetter.get_batches()), 1)
        self.waitTillAssertion(lambda: len(LotsGetter.get_lots()) == 1)

        # test getting list of robots
        msg = CMDMessage(category=CMDCategory.ROBOT, cmd='get_list')
        self.client.send_json(msg.to_json())
        reply = self.client.recv_json()
        stations_dict = json.loads(reply)
        self.assertEqual(len(stations_dict['names']), 1)
        self.assertEqual(len(stations_dict['ids']), 1)

        # test getting list of stations
        msg = CMDMessage(category=CMDCategory.STATION, cmd='get_list')
        self.client.send_json(msg.to_json())
        reply = self.client.recv_json()
        stations_dict = json.loads(reply)
        self.assertEqual(len(stations_dict['names']), 1)
        self.assertEqual(len(stations_dict['ids']), 1)

        # construct station proc handler
        station_type = stations_dict['names'][0]
        station_id = stations_dict['ids'][0]

        station = StationsGetter.get_station(station_id, station_type)
        station_proc_handler = StationProcessHandler(station)

        # process lots in station
        self.waitTillAssertion(lambda: station.free_lot_capacity == 0)
        # tick to start processing lots
        station_proc_handler.handle()
        station_proc_handler.handle()
        station_proc_handler.handle()
        self.waitTillAssertion(lambda: station.free_lot_capacity == 1)

        # wait till lot is in need of removal
        output_state = StateGetter.get_output_state()
        self.waitTillAssertion(lambda: output_state.get_lots_num(LotStatus.NEED_REMOVAL) == 1)

        # test lot removal
        msg = CMDMessage(category=CMDCategory.WORKFLOW, cmd='remove_all_lots')
        self.client.send_json(msg.to_json())
        self.waitTillAssertion(lambda: output_state.get_lots_num(LotStatus.NEED_REMOVAL) == 0)
