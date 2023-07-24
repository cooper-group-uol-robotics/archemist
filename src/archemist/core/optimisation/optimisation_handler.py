import time
from archemist.core.persistence.recipe_files_watchdog import RecipeFilesWatchdog
from pathlib import Path
from threading import Thread
import pandas as pd
from datetime import datetime

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
        

    def update_optimisation_data(self, _values_from_optimizer):
        self._optimized_values = _values_from_optimizer

    def watch_batch_complete(self):
        # get completed batches using the function get_completed_batches from state.py
        # send it to optimisation base and
        # update optimisation data
        completed_batches = self._state.get_completed_batches()
        batch_ids = []
        for batch in completed_batches:
            if batch not in self._optimization_state.batches_seen:
                result_data_dict = batch.extract_samples_op_data(self._opt_update_dict)
                result_data_pd = pd.DataFrame(result_data_dict)
                self._optimizer.update_model(result_data_pd)
                batch_ids.append(batch.id)
        self._optimization_state.batches_seen.extend(batch_ids)


    def watch_recipe_queue(self, recipe_generator):
        # add a max_recipe field in the config.yaml
        # check recipe que, if no recipe add new recipes based on number mentioned in config
        # the new recipes are created based on the recipe_generator 
        recipes_queue = list(self._state.recipes_queue.__iter__())
        recipe_iteration = 0
        while self._max_number_of_recipes > len(recipes_queue):
            recipe_name = f'{self._recipe_name}_{datetime.now()}.yaml'
            recipe_generator.generate_recipe(self._optimized_values[recipe_iteration], recipe_name)
            recipe_iteration += 1
            time.sleep(1)
 
    
    def start(self):
        self._watch_optimization_thread = Thread(target=self.watch_batch_complete())
        self._watch_recipe_thread = Thread(target=self.watch_recipe_queue(self._recipe_generator))
        self._watch_optimization_thread.start()
        self._watch_recipe_thread.start()

        