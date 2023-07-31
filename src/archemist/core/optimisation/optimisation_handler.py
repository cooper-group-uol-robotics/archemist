import time
from typing import Type
from threading import Thread
import pandas as pd
from archemist.core.optimisation.optimisation_records import OptimisationRecords
from archemist.core.optimisation.optimiser_base import OptimiserBase
from archemist.core.state.state import State
from archemist.core.optimisation.recipe_generator import RecipeGenerator

class OptimisationHandler:
    def __init__(self, optimiser: Type[OptimiserBase], optimisation_records: OptimisationRecords, state: State,
                 recipe_generator:RecipeGenerator) -> None:
        self._optimiser = optimiser
        self._optimisation_records = optimisation_records
        self._objective_variable = optimisation_records.objective_variable
        self._state = state
        self._recipe_generator = recipe_generator
        self._is_first_run = optimisation_records.stored_optimisation_obj is None


        self._optimized_values = []
        self._watch_optimization_thread = Thread(target=self.watch_batch_complete, daemon=True)
        self._watch_recipe_thread = Thread(target=self.watch_recipe_queue, daemon=True)

    def watch_batch_complete(self):
        # get completed batches using the function get_completed_batches from state.py
        # send it to optimisation base and
        # update optimisation data
        while True:
            completed_batches = self._state.get_completed_batches()
            for batch in completed_batches:
                if batch.id not in self._optimisation_records.batches_seen:
                    decision_vars_dict = batch.extract_samples_op_data(self._optimisation_records.decision_variables)
                    result_data_dict = batch.extract_samples_op_data(self._optimisation_records.objective_variable)
                    combine_dict = {**decision_vars_dict, **result_data_dict}
                    result_data_pd = pd.DataFrame(combine_dict)
                    updated_model = self._optimiser.update_model(result_data_pd)
                    self._optimisation_records.update_stored_optimisation_model(updated_model)
                    if self._is_first_run:
                        self._is_first_run = False
                    self._optimisation_records.add_to_seen_batches(batch.id)
                    self._logOptHandler(f"model updated with batch {batch.id} with recipe id {batch.recipe.id}")
                    self._logOptHandler("------ start of update data ------")
                    print(str(result_data_pd))
                    self._logOptHandler("------ end of update data ------")
            time.sleep(1)


    def watch_recipe_queue(self):
        # add a max_recipe field in the config.yaml
        # check recipe que, if no recipe add new recipes based on number mentioned in config
        # the new recipes are created based on the recipe_generator
        if self._is_first_run:
            self._logOptHandler(f"First optimisation run. Creating new {self._optimisation_records.max_recipe_count} recipes")
            for _ in range(self._optimisation_records.max_recipe_count):
                _optimized_values = self._optimiser.generate_random_values()
                self._recipe_generator.generate_recipe(_optimized_values)
        while True:
            if len(self._state.recipes_queue) == 0 and not self._is_first_run:
                self._logOptHandler(f"recipes queue is empty. Creating new {self._optimisation_records.max_recipe_count} recipes")
                for _ in range(self._optimisation_records.max_recipe_count):
                    _optimized_values = self._optimiser.generate_batch()
                    self._recipe_generator.generate_recipe(_optimized_values)
            time.sleep(1)

    def start(self):
        self._watch_recipe_thread.start()
        self._watch_optimization_thread.start()

    def _logOptHandler(self, message: str):
        print(f'OptimisationHandler: ' + message)

        