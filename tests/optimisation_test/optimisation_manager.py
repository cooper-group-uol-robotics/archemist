from archemist.core.persistence.yaml_handler import YamlHandler
from optimisation_model import OptimisationModel
from optimiser_base import OptimizerBase
from bayesopt_optimiser import BayesOptOptimizer
from optimisation_handler import OptimizationHandler
from optimisation_state import OptimizationState
from object_factory import OptimizationFactory
from recipe_generator import RecipeGenerator
from pathlib import Path


class OptimisationManager():

    def __init__(self, workflow_dir: str) -> None:
        self._config_file = Path.joinpath(workflow_dir, "config_files/optimization_config.yaml")
        self._recipes_dir = Path.joinpath(workflow_dir, "recipes")
        self._template_dir = Path.joinpath(self._recipes_dir, "template/algae_bot_recipe_template.yaml")
        self._config_dict = YamlHandler.loadYamlFile(self._config_file)
        self._recipe_name = "algae_bot_recipe" #TODO pass from where? config file?
        self._recipe_generator = RecipeGenerator(self._template_dir, self._recipes_dir)

        # optimization constructor
        self._construct_optimizer_from_config_file(self._config_dict)
    
        # Handler
        if self._is_recipe_template_available():
            self._handler = OptimizationHandler(self._recipes_dir, self._max_recipe_count, self._optimizer, self._optimization_state,self._recipe_name)
            _values_from_optimizer = []
            for recipe in range(self._max_recipe_count):
                if self._is_recipe_dir_empty:
                    _values_from_optimizer.append(self._optimizer.generate_random_values())
                else:
                    _values_from_optimizer.append(self._optimizer.generate_batch())
            self._handler.update_optimisation_data(_values_from_optimizer)
        else:
            raise Exception('template file is missing')
        self._handler.start(self._recipe_generator)
        
    def _construct_optimizer_from_config_file(self, config_dict: dict):
        if 'optimizer' in config_dict:
            self._optimization_state = OptimizationFactory.create_from_dict(config_dict['optimizer'])
            self._optimizer = BayesOptOptimizer(self._optimization_state, config_dict)
            self._max_recipe_count = self._optimization_state.max_recipe_count
        else:
            raise Exception('Invalid optimization config file')

    def _is_recipe_template_available(self):
        _template_path = Path(self._template_dir)
        return _template_path.is_file()
    
    def _is_recipe_dir_empty(self):
        directory = Path(self._recipes_dir)
        return not any(directory.iterdir())

if __name__ == '__main__':
    cwd_path = Path.cwd()
    print(cwd_path)
    workflow_dir = Path.joinpath(cwd_path, "tests/optimisation_test")
    opt_mgr = OptimisationManager(workflow_dir)
