from archemist.core.persistence.yaml_handler import YamlHandler
from optimiser_base import OptimizerBase
import pandas as pd
from pathlib import Path
import importlib
    
class OptimisationManager():
    
    def __init__(self,
                 config_file: str,
                 batch_size: int = 15,) -> None:
        self._config_dict = YamlHandler.load_config_file(config_file)
        self._batch_size = batch_size
        bound = self._generate_bound_from_config()
        self._component_keys = bound.keys() 
        #self.target_name = None  
        self.model = self._generate_model(pbounds=bound)
        self._probed_points = pd.DataFrame([])
    
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
        module = importlib.import_module(self._config_dict['optimizer']['module'])
        optimizer_class = getattr(module, self._config_dict['optimizer']['class'])
        return optimizer_class(**self._config_dict['optimizer']['hyperparameters'])

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

if __name__ == "main":
    test_data = pd.DataFrame([[3,4], [1, 2]], columns=['x', 'y'])
    opt = OptimisationManager('test_config.yaml')
    opt.update_model(test_data)
    cwd_path = Path.cwd()
    path_to_template_recipe = Path.joinpath(cwd_path, "tests/optimisation_test/algae_bot_recipe_template.yaml")
    opt.is_recipe_template_available(path_to_template_recipe)

