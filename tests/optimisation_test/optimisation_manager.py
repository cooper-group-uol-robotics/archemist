from archemist.core.persistence.yaml_handler import YamlHandler
from optimisation_model import OptimisationModel
from optimiser_base import OptimizerBase
import pandas as pd
from pathlib import Path
import importlib

class OptimizationState:
    def __init__(self, optimization_model: OptimisationModel) -> None:
        self._model = optimization_model

    @classmethod
    def from_dict(cls, state_dict: dict={}):
        model = OptimisationModel()
        model.optimizer_module = state_dict['module']
        model.optimizer_class = state_dict['class']
        model.optimizer_hyperparameters = state_dict['hyperparameters']
        model.allowed_recipe_count = state_dict['allowed_recipe_count']
        model.save()
        return cls(model)
    
    @property
    def optimizer_module(self) -> str:
        return self._model.optimizer_module
    
    @property
    def optimizer_class(self) -> str:
        return self._model.optimizer_class
    
    @property
    def optimizer_hyperparameters(self):
        return self._model.optimizer_hyperparameters
    
    @property
    def allowed_recipe_count(self) -> int:
        return self._model.allowed_recipe_count
    
class OptimisationManager():
    
    def __init__(self,
                 config_file: str,
                 batch_size: int = 15,) -> None:
        self._config_dict = YamlHandler.loadYamlFile(config_file)
        print(self._config_dict)
        self._batch_size = batch_size
        bound = self._generate_bound_from_config()
        self._component_keys = bound.keys() 
        self.target_name = None  
        self._optimizer = self.construct_optimiser_from_config_file(self._config_dict)
        #self.model = self._generate_model(pbounds=bound)
        self._probed_points = pd.DataFrame([])

    def construct_optimiser_from_config_file(self, config_dict:dict):
        return OptimizationState.from_dict(config_dict['optimizer'])
    
    def recipe_count(self):
        return self._optimizer.allowed_recipe_count
    
    def update_model(self, data: pd.DataFrame, **kwargs):
        """
        Record probed points and update optimization model.
        :param data:

        """
        self._probed_points += data
        params, targets = self._pandas_to_params_targets(data)
        for param, target in zip(params, targets):
            self.model.register(param, target)
    
    def is_recipe_template_available(self, _template_path):
        _template_path = Path(_template_path)
        return _template_path.is_file()

    #internal functions
    
    def _generate_bound_from_config(self):
        constraint = {}
        for key in self._config_dict['experiment']['components']:
            constraint[key] = (self._config_dict['experiment']['components'][key]['lower_bound'],
                               self._config_dict['experiment']['components'][key]['upper_bound'])
        return constraint

    def _generate_model(self):
        """
        Generate optimization model from config file based on the 'optimizer' part.
        If the model requires constraint information, please pass it via kwargs.
        """
        module = importlib.import_module(self._optimizer.optimizer_module)
        optimizer_class = getattr(module, self._optimizer.optimizer_class)
        return optimizer_class(self._optimizer.optimizer_hyperparameters)

    def _pandas_to_params_targets(self, data: pd.DataFrame):
        """
        Split experiment result dataframe into dictionary format params and target array.
        """
        # get optimization target name
        if self.target_name is None:
            target_name = [i for i in data.columns if i not in self._component_keys]
            assert len(target_name) == 1, 'Require only one unseen column as optimization target'
            self.target_name = target_name[0]
        else:
            assert self.target_name in data.columns, 'Optimization target not in result data, please check dataframe ' \
                                                     'format.'

        targets = data[self.target_name].to_numpy()
        data = data.drop(columns=self.target_name)
        params = []
        for row_index in range(len(data)):
            params.append(data.iloc[row_index].to_dict())
        return params, targets

# if __name__ == "__main__":
    # test_data = pd.DataFrame([[3,4], [1, 2]], columns=['x', 'y'])
    # cwd_path = Path.cwd()
    # path_to_config_file = Path.joinpath(cwd_path, "tests/optimisation_test/optimization_config.yaml")
    # opt = OptimisationManager(path_to_config_file)
    # opt.update_model(test_data)
    # cwd_path = Path.cwd()
    # path_to_template_recipe = Path.joinpath(cwd_path, "tests/optimisation_test/algae_bot_recipe_template.yaml")
    # if opt.is_recipe_template_available(path_to_template_recipe) == True:
    #     print ('True')
    # else:
    #     print('false')