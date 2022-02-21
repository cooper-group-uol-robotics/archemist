from archemist.processing.prcessor import WorkflowManager
from archemist.persistence.persistenceManager import PersistenceManager
from archemist.util.location import Location
from pathlib import Path
from time import sleep

if __name__ == '__main__':
    resources_dir = Path.cwd().parent.joinpath('state/resources')
    config_file_path = resources_dir.joinpath('testing_config_file.yaml')
    recipe_file_path = resources_dir.joinpath('testing_recipe.yaml')

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

