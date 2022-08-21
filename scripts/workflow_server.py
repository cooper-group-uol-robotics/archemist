from time import sleep
from archemist.processing.prcessor import WorkflowManager
from archemist.persistence.persistenceManager import PersistenceManager
from archemist.persistence.yamlHandler import YamlHandler
from archemist.util.location import Location
from pathlib import Path
from datetime import datetime
from archemist.state.robots.kukaLBRIIWA import KukaNAVTask, KukaLBRTask, KukaLBRMaintenanceTask
from archemist.state.robot import RobotOutputDescriptor
import zmq

if __name__ == '__main__':
    config_file_name = 'algae_bot_config_file.yaml'
    recipe_file_name = 'algae_bot_test_recipe.yaml'
    db_name = 'algae_bot_test'
    clean_batch_location = Location(25,1,'/InputStation')
    
    current_dir = Path.cwd()
    config_file_path = current_dir.joinpath(f'config_files/{config_file_name}')
    recipe_file_path = current_dir.joinpath(f'recipes/{recipe_file_name}')

    context = zmq.Context()
    socket = context.socket(zmq.PAIR)
    socket.bind('tcp://127.0.0.1:5555')
    # Construct state from config file
    pers_manager = PersistenceManager(db_name)
    state = pers_manager.construct_state_from_config_file(config_file_path)
    # construct the state manager
    wm_manager = WorkflowManager(state)

    now = datetime.now()
    current_time = now.strftime("%H:%M:%S")
    print(f'[{current_time}] starting workflow Processor')
    
    # add clean batch
    batch_id = 0
    state.add_clean_batch(batch_id, 6, clean_batch_location)
    # queue recipe 
    recipe_dict = YamlHandler.loadYamlFile(recipe_file_path)
    wm_manager.queue_recipe(recipe_dict) #TODO this can be done using a watchdog for folder check https://michaelcho.me/article/using-pythons-watchdog-to-monitor-changes-to-a-directory

    # spin
    while True:
        if state.is_batch_complete(batch_id):
            batch_id += 1
            state.add_clean_batch(batch_id, 6, clean_batch_location)
            wm_manager.queue_recipe(recipe_dict)
        try:
            msg = socket.recv_string(flags=zmq.NOBLOCK)
            if msg == 'start':
                if not wm_manager._running:
                    wm_manager.start_processor()
                else:
                    wm_manager.pause_workflow = False
            elif msg == 'pause':
                wm_manager.pause_workflow = True
            elif msg == 'charge':
                wm_manager.queue_robot_op(KukaLBRMaintenanceTask('ChargeRobot',[False,85],RobotOutputDescriptor()))
            elif msg == 'stop_charge':
                wm_manager.queue_robot_op(KukaLBRMaintenanceTask('StopCharge',[False],RobotOutputDescriptor()))
            elif msg == 'resume_app':
                wm_manager.queue_robot_op(KukaLBRMaintenanceTask('resumeLBRApp',[False],RobotOutputDescriptor()))
            elif msg == 'terminate':
                if wm_manager._running:
                    wm_manager.stop_processor()
                    break
        except zmq.ZMQError:
            sleep(0.2)
        except KeyboardInterrupt:
            if wm_manager._running:
                wm_manager.stop_processor()
                break