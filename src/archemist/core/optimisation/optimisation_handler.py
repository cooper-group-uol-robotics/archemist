import time
from archemist.core.persistence.recipe_files_watchdog import RecipeFilesWatchdog
from pathlib import Path
from threading import Thread
import pandas as pd


class OptimizationHandler:
    def __init__(self, recipe_dir, max_recipe_count, optimizer, optimization_state, state, recipe_name, opt_update_dict,recipe_generator) -> None:
        self._recipe_path = recipe_dir
        self._recipe_name = recipe_name
        self._result_path = Path.joinpath(self._recipe_path, "result")
        self._max_number_of_recipes = max_recipe_count
        self._optimizer = optimizer
        self._optimization_state = optimization_state
        self._opt_update_dict = opt_update_dict
        self._state = state
        self._optimized_values = []
        self._recipe_generator = recipe_generator
        self._batches_processed = []

    def update_optimisation_data(self, _values_from_optimizer):
        self._optimized_values = _values_from_optimizer

    def watch_batch_complete(self):
        # get completed batches using the function get_completed_batches from state.py
        # send it to optimisation base and
        # update optimisation data
        completed_batches = self._state.get_completed_batches()
        for batch in completed_batches:
            if batch.recipe.id not in self._batches_processed: 
                result_data_dict = batch.extract_samples_op_data(self._opt_update_dict)
                print(result_data_dict)
                result_data_pd = pd.DataFrame(result_data_dict)
                self._optimizer.update_model(result_data_pd)
                self._batches_processed.append(batch.recipe.id)
                print(self._batches_processed)
            else:
                print(batch.recipe.id, "is already processed")
        # print ('batches processed', self._batches_processed)

    def watch_recipe_queue(self, recipe_generator):
        # add a max_recipe field in the config.yaml
        # check recipe que, if no recipe add new recipes based on number mentioned in config
        # the new recipes are created based on the recipe_generator 
        # if len(recipe_queue) == 0:
        batches_ids = self._batches_processed
        for recipe in range(self._max_number_of_recipes):
            recipe_name = f'{self._recipe_name}_{recipe + 1}.yaml'
            file_increment = 0
            while (self._recipe_path/recipe_name).exists():
                file_increment += 1
                recipe_name = f'{self._recipe_name}_{recipe + 1 + file_increment}.yaml'
            recipe_id = recipe_generator.generate_recipe(self._optimized_values[recipe], recipe_name, batches_ids)
            batches_ids.append(recipe_id)    
    
    def start(self,_watch_optimization_thread, _watch_recipe_thread):
        _watch_recipe_thread.start()
        _watch_optimization_thread.start()
        