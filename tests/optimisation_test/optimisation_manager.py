from archemist.core.persistence.yaml_handler import YamlHandler
from optimisation_model import OptimisationModel
from optimiser_base import OptimizerBase
from bayesopt_optimiser import BayesOptOptimizer
from optimisation_handler import OptimizationHandler
from optimisation_state import OptimizationState
import pandas as pd
from pathlib import Path
from threading import Thread
import importlib

class OptimisationManager():

    def __init__(self,
                 config_file: str,
                 recipe_dir: str,
                 batch_size: int = 15,) -> None:

        self._config_dict = YamlHandler.loadYamlFile(config_file)
        print(self._config_dict)
        self._recipes_dir = recipe_dir
        self._template_dir = Path.joinpath(self._recipes_dir, "template")

        # optimization constructor
        self.construct_optimiser_from_config_file(self._config_dict)
    
        # Handler
        if self.is_recipe_template_available:
            self._handler = OptimizationHandler(self._recipes_dir, self._max_recipe_count, self._template_dir, self._optimizer)
            self._watch_recipe_thread = Thread(target=self._handler.watch_recipe_queue)
            self._watch_optimization_thread = Thread(target=self._handler.watch_batch_complete)
   
        else:
            raise Exception('template file is missing')

        self._probed_points = []  # could be a wrapper class
        self._component_keys = None  # need input for this
        self.target_name = None
        self.model = None
        
    def run(self):
        self._watch_recipe_thread.start()
        self._watch_optimization_thread.start()
        

    def construct_optimiser_from_config_file(self, config_dict: dict):
        self._optimization_model = OptimizationState.from_dict(config_dict['optimizer'])
        self._max_recipe_count = self.recipe_count()
        # todo: the parameters doesn't match optimization base parameter name


        #self._optimizer = BayesOptOptimizer(self._config_dict, self._recipes_dir, self._template_dir)

        self._optimizer = OptimizerBase(opt_module=self._optimization_model.optimizer_module,
                                         opt_class=self._optimization_model.optimizer_class, opt_hyperparameters=self._optimization_model.optimizer_hyperparameters)

    def recipe_count(self):
        return self._optimization_model.max_recipe_count

    def is_recipe_template_available(self):
        _template_path = Path(self._template_dir)
        return _template_path.is_file()
