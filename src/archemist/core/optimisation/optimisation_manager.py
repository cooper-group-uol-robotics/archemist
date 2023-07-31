from archemist.core.persistence.yaml_handler import YamlHandler
from archemist.core.optimisation.optimisation_handler import OptimisationHandler
from archemist.core.optimisation.optimisation_records import OptimisationRecords
from archemist.core.state.state import State
from archemist.core.persistence.object_factory import OptimisationFactory
from .recipe_generator import RecipeGenerator
from pathlib import Path

class OptimisationManager():

    def __init__(self, workflow_dir: Path, state: State, existing_db: bool = False) -> None:
        # construct optimisation state
        config_file = workflow_dir.joinpath("config_files/optimization_config.yaml")
        config_dict = YamlHandler.loadYamlFile(config_file)
        if existing_db:
            self._optimisation_records = OptimisationRecords.from_existing_model()
        else:
            self._optimisation_records = OptimisationRecords.from_dict(config_dict)

        # construct optimiser
        self._optimiser = OptimisationFactory.create_from_records(self._optimisation_records)

        # construct recipe generator
        self._state = state
        recipes_dir = workflow_dir.joinpath("recipes")
        template_name = self._optimisation_records.recipe_template_name
        template_dir = recipes_dir.joinpath(f"template/{template_name}.yaml")
        self._recipe_generator = RecipeGenerator(template_dir, recipes_dir, self._optimisation_records.generated_recipes_prefix, self._state)
            
        # construct handler
        self._handler = OptimisationHandler(self._optimiser, self._optimisation_records, self._state, self._recipe_generator)
        
    def start_optimisation(self):
        self._handler.start()