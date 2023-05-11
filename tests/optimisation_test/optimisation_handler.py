# thread to watch recipe queue
# thread to watch completed 
import time
#from archemist.core.optimisation.recipe_generator import RecipeGenerator
from archemist.core.persistence.recipe_files_watchdog import RecipeFilesWatchdog
from optimisation_manager import OptimizationState, OptimisationManager
from recipe_generator import RecipeGenerator
from pathlib import Path
from threading import Thread
import pandas as pd

class OptimizationHandler:
    def __init__(self, recipe_dir) -> None:
        self.path_to_config_file = Path.joinpath(Path.cwd(), "tests/optimisation_test/optimization_config.yaml")
        self.path_to_template_recipe = Path.joinpath(Path.cwd(), "tests/optimisation_test/algae_bot_recipe_template.yaml")
        self._opt_mgr = OptimisationManager(self.path_to_config_file)
        self.allowed_number_of_recipes = int(self._opt_mgr.recipe_count())
        self.recipe_path = recipe_dir

        self._watch_optimization_thread = Thread(target=self._watch_batch_complete)
        self._watch_recipe_thread = Thread(target=self._watch_recipe_queue)
        self._watch_optimization_thread.start()
        self._watch_recipe_thread.start()


        ### variables to be replaced with functions
        optimised_parameters = {'water': [29.0, 32.0, 34.0, 37.0, 40.0, 43.0], 'dye_A': [0.3, 0.33, 0.35, 0.38, 0.41, 0.44], 'dye_B': [0.31, 0.34, 0.36, 0.39, 0.42, 0.45]}
        self.opt_pd = pd.DataFrame(optimised_parameters)

    def _watch_batch_complete(self):
        #get completed batches using the function get_completed_batches from state.py
        #send it to optimisation base and 
        #update optimisation data
        for i in range(6):
            print(f'{i}th iteration')

    def _watch_recipe_queue(self):
        # add a max_recipe field in the config.yaml
        # check recipe que, if no recipe add new recipes based on number mentioned in config
        # the new recipes are created based on the recipe_generator 
        self.recipe_watcher = RecipeFilesWatchdog(self.recipe_path)
        self.recipe_watcher.start()
        self._recipe_queue = self.recipe_watcher.recipes_queue
        if len(self._recipe_queue) == 0:
            for recipe in range(self.allowed_number_of_recipes):
                self.recipe_name = Path.joinpath(self.recipe_path, f"algae_bot_recipe_{recipe+1}.yaml")
                new_recipe = RecipeGenerator(self.path_to_template_recipe, self.recipe_name)
                new_recipe.generate_recipe(self.opt_pd) 
        time.sleep(1)

if __name__ == '__main__':
    cwd_path = Path.cwd()
    print(cwd_path)
    recipes_dir = Path.joinpath(cwd_path, "tests/optimisation_test/recipes")
    OptimizationHandler(recipes_dir)
