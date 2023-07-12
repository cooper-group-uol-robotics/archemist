import time
import yaml
from archemist.core.persistence.recipe_files_watchdog import RecipeFilesWatchdog
from archemist.core.persistence.persistence_manager import PersistenceManager
from archemist.core.state.state import State
from archemist.core.state.batch import Batch
from recipe_generator import RecipeGenerator
from pathlib import Path
from threading import Thread
import pandas as pd


class OptimizationHandler:
    def __init__(self, recipe_dir, max_recipe_count, optimizer, recipe_name) -> None:
        self._recipe_path = recipe_dir
        self._recipe_name = recipe_name
        self._result_path = Path.joinpath(self._recipe_path, "result")
        self._max_number_of_recipes = max_recipe_count
        self._optimizer = optimizer
        self._opt_update_dict = {'CSCSVJobOpDescriptor':{'dispense_info':{}},
            'SampleColorOpDescriptor': {'red_intensity': ''}# ,
                                                            #  'green_intensity': '',
                                                            #  'blue_intensity': ''}
                                 }

        host = 'mongodb://localhost:27017'
        pm = PersistenceManager(host, 'algae_bot_workflow')
        self._state = pm.construct_state_from_db()

    def update_optimisation_data(self, _values_from_optimizer):
        self._optimized_values = _values_from_optimizer

    def watch_batch_complete(self):
        # get completed batches using the function get_completed_batches from state.py
        # send it to optimisation base and
        # update optimisation data
        completed_batches = self._state.get_completed_batches()
        for batch in completed_batches:
            result_data_dict = batch.extract_samples_op_data(self._opt_update_dict) # pd.DataFrame.from_dict(result)
            result_data_pd = pd.DataFrame(result_data_dict)
            print(result_data_pd)
            self._optimizer.update_model(result_data_pd)

    def watch_recipe_queue(self, recipe_generator):
        # add a max_recipe field in the config.yaml
        # check recipe que, if no recipe add new recipes based on number mentioned in config
        # the new recipes are created based on the recipe_generator 
        recipe_watcher = RecipeFilesWatchdog(self._recipe_path)
        recipe_watcher.start()
        recipe_queue = recipe_watcher.recipes_queue
        # if len(recipe_queue) == 0:
        for recipe in range(self._max_number_of_recipes):
            recipe_name = f'{self._recipe_name}_{recipe + 1}.yaml'
            recipe_generator.generate_recipe(self._optimized_values[recipe], recipe_name)
            time.sleep(1)
    
    def start(self, recipe_generator):
        self._watch_optimization_thread = Thread(target=self.watch_batch_complete())
        self._watch_recipe_thread = Thread(target=self.watch_recipe_queue(recipe_generator ))
        self._watch_optimization_thread.start()
        self._watch_recipe_thread.start()
