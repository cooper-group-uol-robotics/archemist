from archemist.processing.prcessor import WorkflowManager
from archemist.persistence.persistenceManager import PersistenceManager
from archemist.util.location import Location
from pathlib import Path

if __name__ == '__main__':
    current_dir = Path.cwd()
    config_file_path = current_dir.joinpath('config_files/solubility_screening_config.yaml')
    recipe_file_path = current_dir.joinpath('recipes/solubility_screening_recipe.yaml')

    try:
        
        # Construct state from config file
        pers_manager = PersistenceManager('solubility_screening')
        state = pers_manager.construct_state_from_config_file(config_file_path)
        # construct the state manager
        wm_manager = WorkflowManager(state)
        wm_manager.start_processor()

    except KeyboardInterrupt:
        wm_manager.stop_processor()
