from time import sleep
from archemist.processing.prcessor import WorkflowManager
from archemist.persistence.persistenceManager import PersistenceManager
from archemist.persistence.recipe_files_watchdog import RecipeFilesWatchdog
from archemist.persistence.yamlHandler import YamlHandler
from archemist.util.location import Location
from pathlib import Path
from datetime import datetime
from archemist.state.robots.kukaLBRIIWA import KukaNAVTask, KukaLBRTask, KukaLBRMaintenanceTask
from archemist.state.robot import RobotOutputDescriptor
import zmq

if __name__ == '__main__':
    # working_dir = Path('C:\\Users\\ACL_KUKA2\\algae_bot_workflow')
    working_dir = Path('/home/gilgamish/algae_bot_workflow')
    config_file_name = 'chemspeed_testing_config_file.yaml'
    db_name = 'chemspeed_testing'
    clean_batch_location = Location(25,1,'/InputStation')
    loop_recipe = False
    recipe_to_loop = None
    loop_batch = False
    batch_id = 0
    loop_batch_added = False
    
    config_file_path = working_dir.joinpath(f'config_files/{config_file_name}')
    recipes_dir_path = working_dir.joinpath(f'recipes')

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
    print(f'[{current_time}] Started workflow Server')

    recipes_watcher = RecipeFilesWatchdog(recipes_dir_path)
    recipes_watcher.start()


    # recipe_index = 0
    # init = False
    
    # spin
    while True:

        if loop_recipe:
            if recipe_to_loop is None:
                if recipes_watcher.recipes_queue:
                    recipe_file_path = recipes_watcher.recipes_queue.pop()
                    recipe_to_loop = YamlHandler.loadYamlFile(recipe_file_path)
            elif len(wm_manager.recipes_queue) == 0:
                wm_manager.queue_recipe(recipe_to_loop)
        else:
            while recipes_watcher.recipes_queue:
                recipe_file_path = recipes_watcher.recipes_queue.pop()
                recipe_dict = YamlHandler.loadYamlFile(recipe_file_path)
                wm_manager.queue_recipe(recipe_dict)
                print(f'new recipe file queued')

        if loop_batch:
            if not loop_batch_added:
                state.add_clean_batch(batch_id, 6, clean_batch_location)
                print(f'Batch (id: {batch_id}) is added')
                loop_batch_added = True
            elif state.is_batch_complete(batch_id):
                batch_id += 1
                state.add_clean_batch(batch_id, 6, clean_batch_location)
                print(f'Batch (id: {batch_id}) is added')


        # if init:
        #     recipe_file_path = Path(f'C:\\Users\\ACL_KUKA2\\algae_recipes\\algae_bot_recipe_{recipe_index}.yaml')
        #     recipe_dict = YamlHandler.loadYamlFile(recipe_file_path)
        #     wm_manager.queue_recipe(recipe_dict)
        #     print(f'Recipe number {recipe_index} is being processed')
        #     state.add_clean_batch(batch_id, 6, clean_batch_location)
        #     print(f'Batch (id: {batch_id}) is added')
        #     init = False
        # if state.is_batch_complete(batch_id):
        #     recipe_index += 1
        #     batch_id += 1
        #     recipe_file_path = Path(f'C:\\Users\\ACL_KUKA2\\algae_recipes\\algae_bot_recipe_{recipe_index}.yaml')   
        #     recipe_dict = YamlHandler.loadYamlFile(recipe_file_path)
        #     input('Enter to continue a letter and then press enter to continue')
        #     wm_manager.queue_recipe(recipe_dict)
        #     print(f'Recipe number {recipe_index} is being processed')
        #     state.add_clean_batch(batch_id, 6, clean_batch_location)
        #     print(f'Batch (id: {batch_id}) is added')
        # if recipe_index == 10:
        #     print('resetting recipe index back to 0')
        #     recipe_index = 0
        #     batch_id += 1
        #     init = True

        try:
            msg = socket.recv_string(flags=zmq.NOBLOCK)
            if msg == 'start':
                if not wm_manager._running:
                    wm_manager.start_processor()
                    init = True
                else:
                    wm_manager.pause_workflow = False
            elif msg == 'pause':
                wm_manager.pause_workflow = True
            elif msg == 'add_batch':
                state.add_clean_batch(batch_id, 6, clean_batch_location)
                batch_id += 1
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