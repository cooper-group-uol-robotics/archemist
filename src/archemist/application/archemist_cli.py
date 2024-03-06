from __future__ import print_function, unicode_literals
from typing import Dict
from archemist.application.cmd_message import CMDCategory, CMDMessage
from PyInquirer import prompt
from time import sleep
import zmq
import json


class ArchemistCLI:
    def __init__(self) -> None:
        context = zmq.Context()
        self._client = context.socket(zmq.PAIR)
        self._client.connect('tcp://127.0.0.1:5555')

    def run(self):
        main_menu = [
            {
                'type': 'list',
                'name': 'main_menu',
                'message': 'Please select from one of the options below:',
                'choices': ['Start/Resume', 'Pause', 'Add clean batch', 'Clear need to remove lots', 'Stations', 'Robots', 'Terminate']
            }
        ]
        while True:
            try:
                selection = prompt(main_menu)
                if selection['main_menu'] == 'Start/Resume':
                    self._log_client('staring workflow')
                    msg = CMDMessage(
                        category=CMDCategory.WORKFLOW, cmd='start')
                    self._client.send_json(msg.to_json())
                elif selection['main_menu'] == 'Pause':
                    self._log_client('pausing workflow')
                    msg = CMDMessage(
                        category=CMDCategory.WORKFLOW, cmd='pause')
                    self._client.send_json(msg.to_json())
                elif selection['main_menu'] == 'Add clean batch':
                    self._log_client('adding clean batch to the workflow')
                    msg = CMDMessage(
                        category=CMDCategory.WORKFLOW, cmd='add_batch')
                    self._client.send_json(msg.to_json())
                elif selection['main_menu'] == 'Clear need to remove lots':
                    self._log_client('clearing all need to remove lots')
                    msg = CMDMessage(
                        category=CMDCategory.WORKFLOW, cmd='remove_all_lots')
                    self._client.send_json(msg.to_json())
                elif selection['main_menu'] == 'Stations':
                    msg = CMDMessage(
                        category=CMDCategory.STATION, cmd='get_list')
                    self._client.send_json(msg.to_json())
                    reply = self._client.recv_json()
                    stations_dict = json.loads(reply)
                    self._process_list_menu(stations_dict, CMDCategory.STATION)
                elif selection['main_menu'] == 'Robots':
                    msg = CMDMessage(
                        category=CMDCategory.ROBOT, cmd='get_list')
                    self._client.send_json(msg.to_json())
                    reply = self._client.recv_json()
                    robots_dict = json.loads(reply)
                    self._process_list_menu(robots_dict, CMDCategory.ROBOT)
                elif selection['main_menu'] == 'Terminate':
                    self._log_client('Terminating workflow')
                    msg = CMDMessage(
                        category=CMDCategory.WORKFLOW, cmd='terminate')
                    self._client.send_json(msg.to_json())
                    break
                sleep(.1)
            except KeyboardInterrupt:
                self._client.close()
                break

    def _process_list_menu(self, objects_dict: Dict, category: CMDCategory):
        list_menu = {
            'type': 'list',
            'name': 'list_menu',
            'message': 'Please select from one of the options below:',
            'choices': objects_dict['names'] + ['Return']
        }
        selection = prompt(list_menu)
        object_name = selection['list_menu']
        if object_name == 'Return':
            pass
        else:
            index = objects_dict['names'].index(object_name)
            object_id = objects_dict['ids'][index]
            if category == CMDCategory.STATION:
                self._process_indivdual_station_menu(object_name, object_id)
            else:
                self._process_indivdual_robot_menu(object_name, object_id)

    def _process_indivdual_station_menu(self, station_name: str, station_id: int):
        station_menu = {
            'type': 'list',
            'name': 'station_menu',
            'message': 'Please select from one of the stations below:',
            'choices': ['Repeat assigned op', 'Skip assigned op', 'Return']
        }
        selection = prompt(station_menu)
        if selection['station_menu'] == 'Repeat assigned op':
            self._log_client(
                f'repeating assigned op for station {station_name} with id: {station_id}')
            msg = CMDMessage(category=CMDCategory.STATION,
                             cmd='repeat_op', params=[station_name, station_id])
            self._client.send_json(msg.to_json())
        elif selection['station_menu'] == 'Skip assigned op':
            self._log_client(
                f'skipping assigned op for station {station_name} with id: {station_id}')
            msg = CMDMessage(category=CMDCategory.STATION,
                             cmd='skip_op', params=[station_name, station_id])
            self._client.send_json(msg.to_json())
        elif selection['station_menu'] == 'Return':
            pass

    def _process_indivdual_robot_menu(self, robot_name: str, robot_id: int):
        robot_menu = {
            'type': 'list',
            'name': 'robot_menu',
            'message': 'Please select from one of the stations below:',
            'choices': ['Repeat assigned op', 'Skip assigned op', 'Return']
        }
        selection = prompt(robot_menu)
        if selection['robot_menu'] == 'Repeat assigned op':
            self._log_client(
                f'repeating assigned op for station {robot_name} with id: {robot_id}')
            msg = CMDMessage(category=CMDCategory.ROBOT,
                             cmd='repeat_op', params=[robot_name, robot_id])
            self._client.send_json(msg.to_json())
        elif selection['robot_menu'] == 'Skip assigned op':
            self._log_client(
                f'skipping assigned op for station {robot_name} with id: {robot_id}')
            msg = CMDMessage(category=CMDCategory.ROBOT,
                             cmd='skip_op', params=[robot_name, robot_id])
            self._client.send_json(msg.to_json())
        elif selection['robot_menu'] == 'Return':
            pass

    def _log_client(self, message: str):
        print(f'[{self.__class__.__name__}]: {message}')
