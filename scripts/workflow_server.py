from time import sleep
from archemist.processing.prcessor import WorkflowManager
from archemist.persistence.persistenceManager import PersistenceManager
from archemist.persistence.yamlHandler import YamlHandler
from archemist.util.location import Location
from pathlib import Path
from datetime import datetime
from archemist.state.robots.kukaLBRIIWA import KukaNAVTask, KukaLBRTask
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
    wm_manager.start_processor()

    now = datetime.now()
    current_time = now.strftime("%H:%M:%S")
    print(f'[{current_time}] starting workflow')
    input('press enter to start')
    
    # add clean batch
    batch_id = 0
    state.add_clean_batch(batch_id, 6, clean_batch_location)
    # queue recipe 
    recipe_dict = YamlHandler.loadYamlFile(recipe_file_path)
    wm_manager.queue_recipe(recipe_dict)

    # spin
    while True:
        try:
            msg = socket.recv_string(flag=zmq.NOBLOCK)
            if msg == 'start':
                if not wm_manager._running:
                    wm_manager.start_processor()
                else:
                    wm_manager.resume_workflow_processing()
            elif msg == 'pause':
                wm_manager.pause_workflow_processing()
            elif msg == 'charge':
                wm_manager.queue_robot_op(KukaLBRTask('ChargeRobot',[False,85],Location(-1,-1,''),RobotOutputDescriptor()))
            elif msg == 'stop_charge':
                wm_manager.queue_robot_op(KukaLBRTask('StopCharge',[False],Location(-1,-1,''),RobotOutputDescriptor()))
            elif msg == 'stop_charge':
                wm_manager.queue_robot_op(KukaLBRTask('StopCharge',[False],Location(-1,-1,''),RobotOutputDescriptor()))
            elif msg == 'resume_app':
                wm_manager.queue_robot_op(KukaLBRTask('resumeLBRApp',[False],Location(-1,-1,''),RobotOutputDescriptor()))
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