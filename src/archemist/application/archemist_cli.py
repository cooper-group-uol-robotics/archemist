from __future__ import print_function, unicode_literals
from typing import Dict
from archemist.application.cmd_message import CMDCategory,CMDMessage
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
                'choices': ['Start/Resume', 'Pause','Add clean batch', 'Remove waiting batches', 'Optimisation','Stations', 'Robots', 'Terminate']
            }
        ]
        while True:
            try:
                selection = prompt(main_menu)
                if selection['main_menu'] == 'Start/Resume':
                    print('Staring workflow')
                    msg = CMDMessage(category=CMDCategory.WORKFLOW, cmd='start')
                    self._client.send_json(msg.to_json())
                elif selection['main_menu'] == 'Pause':
                    print('Pausing workflow')
                    msg = CMDMessage(category=CMDCategory.WORKFLOW, cmd='pause')
                    self._client.send_json(msg.to_json())
                elif selection['main_menu'] == 'Add clean batch':
                    print('Adding clean batch to the workflow')
                    msg = CMDMessage(category=CMDCategory.WORKFLOW, cmd='add_batch')
                    self._client.send_json(msg.to_json())
                elif selection['main_menu'] == 'Remove waiting batches':
                    print('Removing waiting batches from the workflow')
                    msg = CMDMessage(category=CMDCategory.WORKFLOW, cmd='manual_batch_removal')
                    self._client.send_json(msg.to_json())
                elif selection['main_menu'] == 'Optimisation':
                    self._process_optimisation_menu()
                elif selection['main_menu'] == 'Stations':
                    msg = CMDMessage(category=CMDCategory.STATION, cmd='get_list')
                    self._client.send_json(msg.to_json())
                    reply = self._client.recv_json()
                    stations_dict = json.loads(reply)
                    self._process_list_menu(stations_dict, CMDCategory.STATION)
                elif selection['main_menu'] == 'Robots':
                    msg = CMDMessage(category=CMDCategory.ROBOT, cmd='get_list')
                    self._client.send_json(msg.to_json())
                    reply = self._client.recv_json()
                    robots_dict = json.loads(reply)
                    self._process_list_menu(robots_dict, CMDCategory.ROBOT)
                elif selection['main_menu'] == 'Terminate':
                    print('Terminating workflow')
                    msg = CMDMessage(category=CMDCategory.WORKFLOW, cmd='terminate')
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
            'choices': ['Repeate assigned op', 'Skip assigned op', 'Return']
            }
        selection = prompt(station_menu)
        if selection['station_menu'] == 'Repeate assigned op':
            print(f'Repeating assigned op for station {station_name} with id: {station_id}')
            msg = CMDMessage(category=CMDCategory.STATION, cmd='repeat_op', params=[station_name,station_id])
            self._client.send_json(msg.to_json())
        elif selection['station_menu'] == 'Skip assigned op':
            print(f'Skipping assigned op for station {station_name} with id: {station_id}')
            msg = CMDMessage(category=CMDCategory.STATION, cmd='skip_op', params=[station_name,station_id])
            self._client.send_json(msg.to_json())
        elif selection['station_menu'] == 'Return':
            pass

    def _process_indivdual_robot_menu(self, robot_name: str, robot_id: int):
        robot_menu = {
            'type': 'list',
            'name': 'robot_menu',
            'message': 'Please select from one of the stations below:',
            'choices': ['Repeate assigned op', 'Skip assigned op', 'Return']
            }
        if robot_name == 'KukaLBRIIWA':
            robot_menu['choices'] = ['Send to charge', 'Stop charging', 'Send LBR resume signal'] + robot_menu['choices']
        selection = prompt(robot_menu)
        if selection['robot_menu'] == 'Repeate assigned op':
            print(f'Repeating assigned op for station {robot_name} with id: {robot_id}')
            msg = CMDMessage(category=CMDCategory.ROBOT, cmd='repeat_op', params=[robot_name,robot_id])
            self._client.send_json(msg.to_json())
        elif selection['robot_menu'] == 'Skip assigned op':
            print(f'Skipping assigned op for station {robot_name} with id: {robot_id}')
            msg = CMDMessage(category=CMDCategory.ROBOT, cmd='skip_op', params=[robot_name,robot_id])
            self._client.send_json(msg.to_json())
        elif selection['robot_menu'] == 'Send to charge':
            print(f'Sending {robot_name} with id: {robot_id} to charge')
            msg = CMDMessage(category=CMDCategory.ROBOT, cmd='charge', params=[robot_name,robot_id])
            self._client.send_json(msg.to_json())
        elif selection['robot_menu'] == 'Stop charging':
            print(f'Sending a stop charge message to {robot_name} with id: {robot_id}')
            msg = CMDMessage(category=CMDCategory.ROBOT, cmd='stop_charge', params=[robot_name,robot_id])
            self._client.send_json(msg.to_json())
        elif selection['robot_menu'] == 'Send LBR resume signal':
            print(f'Sending a resume app message to {robot_name} with id: {robot_id}')
            msg = CMDMessage(category=CMDCategory.ROBOT, cmd='resume_app', params=[robot_name,robot_id])
            self._client.send_json(msg.to_json())
        elif selection['robot_menu'] == 'Return':
            pass
    
    def _process_optimisation_menu(self):
        optim_menu = {
            'type': 'list',
            'name': 'optimisation_menu',
            'message': 'Please select from one of the options below:',
            'choices': ['Start', 'Generate new recipe', 'Return']
            }
        selection = prompt(optim_menu)
        if selection['optimisation_menu'] == 'Start':
            print(f'Starting optimistion manager')
            msg = CMDMessage(category=CMDCategory.WORKFLOW, cmd='start_optim')
            self._client.send_json(msg.to_json())
        elif selection['optimisation_menu'] == 'Generate new recipe':
            print('Generating new recipe')
            msg = CMDMessage(category=CMDCategory.WORKFLOW, cmd='generate_recipe')
            self._client.send_json(msg.to_json())
        elif selection['optimisation_menu'] == 'Return':
            pass