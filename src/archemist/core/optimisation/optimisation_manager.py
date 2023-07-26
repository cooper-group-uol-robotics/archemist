from archemist.core.persistence.yaml_handler import YamlHandler
from archemist.core.models.optimisation_model import OptimisationModel
from .optimiser_base import OptimizerBase
from .bayesopt_optimiser import BayesOptOptimizer
from .optimisation_handler import OptimizationHandler
from archemist.core.state.optimisation_state import OptimizationState
from archemist.core.state.state import State
from archemist.core.persistence.object_factory import OptimizationFactory
from .recipe_generator import RecipeGenerator
from pathlib import Path
import importlib

class OptimisationManager():

    def __init__(self, workflow_dir: str, state: State) -> None:
        # construct optimisation state
        config_file = Path.joinpath(workflow_dir, "config_files/optimization_config.yaml")
        config_dict = YamlHandler.loadYamlFile(config_file)
        self._optimization_state = OptimizationState.from_dict(config_dict['optimizer'])

        # construct optimiser
        self._optimizer = OptimizationFactory.create_from_dict(self._optimization_state,config_dict)

        # construct recipe generator
        self._state = state
        recipes_dir = Path.joinpath(workflow_dir, "recipes")
        template_name = self._optimization_state.recipe_template_name
        template_dir = Path.joinpath(recipes_dir, f"template/{template_name}.yaml")
        self._recipe_generator = RecipeGenerator(template_dir, recipes_dir, self._state)
            
        # construct handler
        self._handler = OptimizationHandler(self._optimizer, self._optimization_state, self._state, self._recipe_generator)
        
    def start_optimisation(self):
        self._handler.start()

