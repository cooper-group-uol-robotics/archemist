from typing import Dict, List
from mongoengine import Document, fields
import pandas as pd

class OptimisationRecordsModel(Document):
    optimiser_module = fields.StringField(required=True)
    optimiser_class = fields.StringField(required=True)
    optimiser_args = fields.DictField(default={})
    objective_variable = fields.DictField(required=True)
    decision_variables = fields.DictField(required=True)
    recipe_template_name = fields.StringField(required=True)
    generated_recipes_prefix = fields.StringField(default="recipe")
    max_recipe_count = fields.IntField(min=1)
    save_opt_model_path = fields.StringField(null=True)
    batches_seen = fields.ListField()

    meta = {'collection': 'optimisation', 'db_alias': 'archemist_state'}

class OptimisationRecords:
    def __init__(self, optimization_model: OptimisationRecordsModel) -> None:
        self._model = optimization_model

    @classmethod
    def from_dict(cls, state_dict: dict):
        model = OptimisationRecordsModel()
        model.optimiser_module = state_dict['optimiser']['module']
        model.optimiser_class = state_dict['optimiser']['class']
        model.optimiser_args = state_dict['optimiser']['args']
        model.max_recipe_count = state_dict['max_recipe_count']
        obj_var_dict = state_dict["objective_variable"]
        model.objective_variable = {obj_var_dict["station_op"]: obj_var_dict["fields"]}
        decision_var_list = state_dict["decision_variables"]
        model.decision_variables = {decision_var["station_op"]: decision_var["fields"] for decision_var in decision_var_list}
        model.recipe_template_name = state_dict['recipe_template_name']
        model.generated_recipes_prefix = state_dict['generated_recipes_prefix']
        if "save_opt_model_path" in state_dict:
            model.save_opt_model_path = state_dict['save_opt_model_path']
        model.save()
        return cls(model)

    @property
    def optimiser_module(self) -> str:
        return self._model.optimiser_module

    @property
    def optimiser_class(self) -> str:
        return self._model.optimiser_class
    
    @property
    def optimiser_args(self) -> Dict:
        return self._model.optimiser_args
    
    @property
    def objective_variable(self) -> Dict[str, str]:
        return self._model.objective_variable
    
    @property
    def decision_variables(self) -> Dict[str, str]:
        return self._model.decision_variables
    
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
    def save_opt_model_path(self) -> str:
        return self._model.save_opt_model_path
    
    @property
    def batches_seen(self) -> List[int]:
        self._model.reload('batches_seen')
        return self._model.batches_seen
    
    def add_to_seen_batches(self, batch_id: int):
        self._model.update(push__batches_seen=batch_id)