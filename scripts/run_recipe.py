from time import sleep
from archemist.processing.prcessor import WorkflowManager
from archemist.persistence.persistenceManager import PersistenceManager
from archemist.persistence.yamlHandler import YamlHandler
from archemist.util.location import Location
from pathlib import Path
from datetime import datetime

if __name__ == '__main__':
    config_file_name = 'dif_demo_testing_config_file.yaml'
    recipe_file_name = 'dif_demo_test_recipe.yaml'
    db_name = 'dif_demo'
    clean_batch_location = Location(2,1,'InputStation/RackHolder')
    
    current_dir = Path.cwd()
    config_file_path = current_dir.joinpath(f'config_files/{config_file_name}')
    recipe_file_path = current_dir.joinpath(f'recipes/{recipe_file_name}')

    try:
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
        state.add_clean_batch(batch_id, 2, clean_batch_location)
        # queue recipe 
        recipe_dict = YamlHandler.loadYamlFile(recipe_file_path)
        wm_manager.queue_recipe(recipe_dict)

        # spin
        while True:
            sleep(1)


    except KeyboardInterrupt:
        wm_manager.stop_processor()
