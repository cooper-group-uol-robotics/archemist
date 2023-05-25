import time
from archemist.core.persistence.recipe_files_watchdog import RecipeFilesWatchdog
from archemist.core.state.state import State
from pathlib import Path
from threading import Thread
import pandas as pd

class OptimizationHandler:
    def __init__(self, new_recipe_generator, max_recipe_count, templete_recipe_dir, recipe_dir) -> None:
        self._new_recipe_generator = new_recipe_generator
        self._templete_recipe_path = templete_recipe_dir
        self._recipe_path = recipe_dir
        self._max_number_of_recipes = int(max_recipe_count)
        print('max', self._max_number_of_recipes)

        ### variables to be replaced with functions
        optimised_parameters = {'water': [29.0, 32.0, 34.0, 37.0, 40.0, 43.0], 'dye_A': [0.3, 0.33, 0.35, 0.38, 0.41, 0.44], 'dye_B': [0.31, 0.34, 0.36, 0.39, 0.42, 0.45]}
        self._opt_pd = pd.DataFrame(optimised_parameters)

    def watch_batch_complete(self):
        #get completed batches using the function get_completed_batches from state.py
        #send it to optimisation base and 
        #update optimisation data
        """
        completed_batches = State().get_completed_batches()
        for batch in completed_batches:
            if batch.recipe_attached:
                recipe = batch.recipe
                
                #code to get the probed points(pd.DataFrame) and feed it to opt_manager.updatemodel
                        
                """
        print("thread 1")

    def watch_recipe_queue(self):
        # add a max_recipe field in the config.yaml
        # check recipe que, if no recipe add new recipes based on number mentioned in config
        # the new recipes are created based on the recipe_generator 
        recipe_watcher = RecipeFilesWatchdog(self._recipe_path)
        recipe_watcher.start()
        recipe_queue = recipe_watcher.recipes_queue
        if len(recipe_queue) == 0:
            for recipe in range(self._max_number_of_recipes):
                _file_name = f"algae_bot_recipe_{recipe+1}.yaml"
                # recipe_name = Path.joinpath(self._recipe_path, f"algae_bot_recipe_{recipe+1}.yaml")
                # new_recipe = RecipeGenerator(self._templete_recipe_path, recipe_name)
                self._new_recipe_generator.generate_recipe(self._opt_pd, _file_name) 
        time.sleep(1)

# if __name__ == '__main__':
#     cwd_path = Path.cwd()
#     print(cwd_path)
#     recipes_dir = Path.joinpath(cwd_path, "tests/optimisation_test/recipes")
#     OptimizationHandler(recipes_dir)
