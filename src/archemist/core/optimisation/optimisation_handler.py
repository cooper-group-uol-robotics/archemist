import time
from pathlib import Path
from threading import Thread
import pandas as pd
from archemist.core.optimisation.optimisation_records import OptimizationRecords
from archemist.core.optimisation.bayesopt_optimiser import BayesOptOptimizer
from archemist.core.state.state import State
from archemist.core.optimisation.recipe_generator import RecipeGenerator

class OptimizationHandler:
    def __init__(self, optimizer: BayesOptOptimizer, optimization_records: OptimizationRecords, state: State, recipe_generator:RecipeGenerator) -> None:
        self._optimizer = optimizer
        self._optimization_records = optimization_records
        self._objective_variable = optimization_records.objective_variable
        self._state = state
        self._recipe_generator = recipe_generator
        self._is_initial_run = recipe_generator.is_recipe_dir_empty()

        self._optimized_values = []
        self._watch_optimization_thread = Thread(target=self.watch_batch_complete())
        self._watch_recipe_thread = Thread(target=self.watch_recipe_queue())

    def watch_batch_complete(self):
        # get completed batches using the function get_completed_batches from state.py
        # send it to optimisation base and
        # update optimisation data
        completed_batches = self._state.get_completed_batches()
        for batch in completed_batches:
            if batch not in self._optimization_records.batches_seen:
                result_data_dict = batch.extract_samples_op_data(self._objective_variable)
                result_data_pd = pd.DataFrame(result_data_dict)
                self._optimizer.update_model(result_data_pd)
                self._optimization_records.batches_seen.append(batch.id)


    def watch_recipe_queue(self):
        # add a max_recipe field in the config.yaml
        # check recipe que, if no recipe add new recipes based on number mentioned in config
        # the new recipes are created based on the recipe_generator 
        recipes_buffer = self._state.recipes_queue
        if len(recipes_buffer) == 0:
            for _ in range(self._optimization_records.max_recipe_count):
                if self._is_initial_run:
                    _optimized_values = self._optimizer.generate_random_values()
                    self._is_initial_run = False
                else:
                    _optimized_values = self._optimizer.generate_batch()
                self._recipe_generator.generate_recipe(_optimized_values)
                time.sleep(1)

    def start(self):
        self._watch_optimization_thread.start()
        self._watch_recipe_thread.start()

        