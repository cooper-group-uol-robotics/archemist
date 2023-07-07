import time
import yaml
from archemist.core.persistence.recipe_files_watchdog import RecipeFilesWatchdog
from archemist.core.state.state import State
from recipe_generator import RecipeGenerator
from pathlib import Path
from threading import Thread
import pandas as pd


class OptimizationHandler:
    def __init__(self, recipe_dir, max_recipe_count, templete_recipe_dir, optimizer) -> None:
        self._recipe_path = recipe_dir
        self._result_path = Path.joinpath(self._recipe_path, "result")
        self._templete_recipe_path = templete_recipe_dir
        self._max_number_of_recipes = max_recipe_count
        self._optimizer = optimizer

        ### variables to be replaced with functions
        optimised_parameters = {'water': [29.0, 32.0, 34.0, 37.0, 40.0, 43.0],
                                'dye_A': [0.3, 0.33, 0.35, 0.38, 0.41, 0.44],
                                'dye_B': [0.31, 0.34, 0.36, 0.39, 0.42, 0.45]}
        self._opt_pd = pd.DataFrame(optimised_parameters)
    
    def update_optimisation_data(self, _random_values_dict):
        self._opt_pd = pd.DataFrame(_random_values_dict)


    def watch_batch_complete(self):
        # get completed batches using the function get_completed_batches from state.py
        # send it to optimisation base and
        # update optimisation data
        completed_batches = State().get_completed_batches()
        for batch in completed_batches:
            if batch.recipe_attached:
                batch_id = batch.id
                result_watcher = RecipeFilesWatchdog(self._result_path)
                result_watcher.start()
                result_queue = result_watcher.recipes_queue
                for result_file in result_queue == 0:
                    with open(result_file) as file:
                        result_dict = yaml.load(file, Loader=yaml.SafeLoader)
                    if result_dict['batch_id'] == batch_id:
                        result = {'output': result_dict['output']}
                        result_data = pd.DataFrame.from_dict(result)
                        self._optimizer.update_model(result_data)
                    else:
                        pass

    def watch_recipe_queue(self):
        # add a max_recipe field in the config.yaml
        # check recipe que, if no recipe add new recipes based on number mentioned in config
        # the new recipes are created based on the recipe_generator 
        recipe_watcher = RecipeFilesWatchdog(self._recipe_path)
        recipe_watcher.start()
        recipe_queue = recipe_watcher.recipes_queue
        if len(recipe_queue) == 0:
            for recipe in range(self._max_number_of_recipes):
                recipe_name = f"algae_bot_recipe_{recipe + 1}.yaml"
                # recipe_name = Path.joinpath(self._recipe_path, f"algae_bot_recipe_{recipe + 1}.yaml")
                new_recipe = RecipeGenerator(self._templete_recipe_path, self._recipe_path)
                new_recipe.generate_recipe(self._opt_pd, recipe_name)
        time.sleep(1)

if __name__ == '__main__':
    cwd_path = Path.cwd()
    print(cwd_path)
    recipes_dir = Path.joinpath(cwd_path, "tests/optimisation_test/recipes")
    OptimizationHandler(recipes_dir)
