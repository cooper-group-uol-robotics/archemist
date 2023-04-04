from archemist.core.persistence.yaml_handler import YamlHandler
from optimiser_base import OptimizerBase
import pandas as pd
from pathlib import Path
import importlib
from optimisation_model import OptimisationModel

class State:
    @classmethod
    def from_dict(cls, state_dict: dict={}):
        model = OptimisationModel()
        model.optimiser_module = state_dict['module']
        model.optimiser_class = state_dict['class']
        model.optimiser_hyperparameters = state_dict['optimiser_hyperparameters']
        model.save()
        return cls(model)
    
class OptimisationManager(OptimizerBase):
    
    def __init__(self,
                 config_file: str,
                 batch_size: int = 15,) -> None:
        self.config = self._read_config(file_path=config_file)
        self.batch_size = batch_size
        bound = self.generate_bound_from_config()
        self.component_keys = bound.keys() 
        #self.target_name = None  

        self.model = self.generate_model(pbounds=bound)
        self._probed_points = pd.DataFrame([])

    def construct_optimiser_from_config_file(self, config_file_path:str):
        return State.from_dict(self.config['optimizer'])
    
    def is_recipe_template_available(self, _template_path):
        _template_path = Path(_template_path)
        return _template_path.is_file()


    #optimization_part
    
    def generate_bound_from_config():
        pass

    def generate_model():
        pass

    def update_model(self, data: pd.DataFrame, **kwargs):
        pass

    def pandas_to_params_targets(self, data: pd.DataFrame):
        pass

    def generate_batch(self, **kwargs) -> pd.DataFrame:
        pass




