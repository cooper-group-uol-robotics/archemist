from archemist.processing.prcessor import WorkflowManager
from archemist.persistence.persistenceManager import PersistenceManager
from archemist.util.location import Location
import os
from time import sleep

if __name__ == '__main__':
    current_dir = os.path.dirname(os.path.realpath(__file__))
    config_file_path = os.path.abspath(os.path.join(current_dir, '../state/resources/testing_config_file.yaml'))
    recipe_file_path = os.path.abspath(os.path.join(current_dir, '../state/resources/testing_recipe.yaml'))

    try:
        
        # Construct state from config file
        pers_manager = PersistenceManager('test')
        state = pers_manager.construct_state_from_config_file(config_file_path)
        # construct the state manager
        wm_manager = WorkflowManager(state)
        wm_manager.start_processor()
        #sleep(0.5)
        #wm_manager.add_batch(69,recipe_file_path, 2, Location(1,7,'/ikaStation/RackHolderFrame'))

    except KeyboardInterrupt:
        wm_manager.stop_processor()
    finally:
        print('workmanager process is terminated')

