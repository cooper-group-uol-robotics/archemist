from archemist.core.persistence.yaml_handler import YamlHandler
from archemist.core.models.optimisation_model import OptimisationModel
from optimiser_base import OptimizerBase
from bayesopt_optimiser import BayesOptOptimizer
from optimisation_handler import OptimizationHandler
from archemist.core.state.optimisation_state import OptimizationState
from archemist.core.state.state import State
from archemist.core.persistence.object_factory import OptimizationFactory
from archemist.core.persistence.persistence_manager import PersistenceManager
from archemist.core.persistence.recipe_files_watchdog import RecipeFilesWatchdog
from recipe_generator import RecipeGenerator
from pathlib import Path
import importlib

class OptimisationManager():

    def __init__(self, workflow_dir: str, state: State) -> None:
        self._config_file = Path.joinpath(workflow_dir, "config_files/optimization_config.yaml")
        self._recipes_dir = Path.joinpath(workflow_dir, "recipes")
        self._config_dict = YamlHandler.loadYamlFile(self._config_file)
        self._opt_update_dict = self._config_dict['optimizer']['optimizer_update_dict']
        self._recipe_name = self._config_dict['experiment']['recipe_name']
        self._template_dir = Path.joinpath(self._recipes_dir, f"template/{self._recipe_name}_template.yaml")
        self._state = state
        
        self._recipe_generator = RecipeGenerator(self._template_dir, self._recipes_dir, self._state)

        # optimization constructor
        self._construct_optimizer_from_config_file(self._config_dict)
    
        # Handler
        self._handler = OptimizationHandler(self._recipes_dir, self._max_recipe_count, self._optimizer, self._optimization_state, self._state, self._recipe_name, self._opt_update_dict, self._recipe_generator)

        if self._is_recipe_template_available():
            _values_from_optimizer = []
            for recipe in range(self._max_recipe_count):
                if self._is_recipe_dir_empty:
                    _values_from_optimizer.append(self._optimizer.generate_random_values())
                else:
                    _values_from_optimizer.append(self._optimizer.generate_batch())
            self._handler.update_optimisation_data(_values_from_optimizer)

        else:
            raise Exception('template file is missing')
        self._handler.start()
        
    def _construct_optimizer_from_config_file(self, config_dict: dict):
        if 'optimizer' in config_dict:
            self._optimization_state = OptimizationState.from_dict(config_dict['optimizer'])
            self._optimizer = OptimizationFactory.create_from_dict(self._optimization_state,config_dict)
            self._max_recipe_count = self._optimization_state.max_recipe_count  
        else:
            raise Exception('Invalid optimization config file')

    def _is_recipe_template_available(self):
        _template_path = Path(self._template_dir)
        return _template_path.is_file()
    
    def _is_recipe_dir_empty(self):
        directory = Path(self._recipes_dir)
        return not any(directory.iterdir())

