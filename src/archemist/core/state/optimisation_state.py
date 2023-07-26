from typing import Dict, List
from archemist.core.models.optimisation_model import OptimisationModel


class OptimizationState:
    def __init__(self, optimization_model: OptimisationModel) -> None:
        self._model = optimization_model

    @classmethod
    def from_dict(cls, state_dict: dict):
        model = OptimisationModel()
        model.optimizer_module = state_dict['module']
        model.optimizer_class = state_dict['class']
        model.optimizer_hyperparameters = state_dict['hyperparameters']
        model.max_recipe_count = state_dict['max_recipe_count']
        obj_var_dict = state_dict["objective_variable"]
        model.objective_variable = {obj_var_dict["station_op"]: obj_var_dict["field"]}
        decision_var_list = state_dict["decision_variables"]
        model.decision_variables = [{decision_var["station_op"]: decision_var["fields"]} for decision_var in decision_var_list]
        model.recipe_template_name = state_dict['recipe_template_name']
        model.generated_recipes_prefix = state_dict['generated_recipes_prefix']
        model.save()
        return cls(model)

    @property
    def optimizer_module(self) -> str:
        return self._model.optimizer_module

    @property
    def optimizer_class(self) -> str:
        return self._model.optimizer_class
    
    @property
    def objective_variable(self) -> Dict[str, str]:
        return self._model.objective_variable
    
    @property
    def decision_variables(self) -> List[Dict[str, str]]:
        return self._model.decision_variables
    
    @property
    def optimizer_hyperparameters(self) -> Dict:
        return self._model.optimizer_hyperparameters
    
    @property
    def recipe_template_name(self) -> str:
        return self._model.recipe_template_name
    
    @property
    def generated_recipes_prefix(self) -> str:
        return self._model.generated_recipes_prefix

    @property
    def max_recipe_count(self) -> int:
        return self._model.max_recipe_count
    
    @property
    def batches_seen(self) -> List[int]:
        self._model.reload('batches_seen')
        return self._model.batches_seen
    