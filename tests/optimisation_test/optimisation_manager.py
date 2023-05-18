from archemist.core.persistence.yaml_handler import YamlHandler
from optimisation_model import OptimisationModel
from optimiser_base import OptimizerBase
from optimisation_handler import OptimizationHandler
import pandas as pd
from pathlib import Path
from threading import Thread
import importlib


class OptimizationState:
    def __init__(self, optimization_model: OptimisationModel) -> None:
        self._model = optimization_model

    @classmethod
    def from_dict(cls, state_dict: dict = {}):
        model = OptimisationModel()
        model.optimizer_module = state_dict['module']
        model.optimizer_class = state_dict['class']
        model.optimizer_hyperparameters = state_dict['hyperparameters']
        model.max_recipe_count = state_dict['max_recipe_count']
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
    def max_recipe_count(self) -> int:
        return self._model.max_recipe_count


class OptimisationManager():

    def __init__(self,
                 config_file: str,
                 recipe_dir: str,
                 templete_dir:str,
                 batch_size: int = 15,) -> None:

        self._config_dict = YamlHandler.loadYamlFile(config_file)
        print(self._config_dict)
        self._recipes_dir = recipe_dir
        self._template_dir = templete_dir

        # optimization constructor
        self._optimizer = self.construct_optimiser_from_config_file(self._config_dict)
    
        # Handler
        if self.is_recipe_template_available:
            self._handler = OptimizationHandler(self._recipes_dir, self._max_recipe_count, self._template_dir)
            self._watch_recipe_thread = Thread(target=self._handler.watch_recipe_queue)
            self._watch_optimization_thread = Thread(target=self._handler.watch_batch_complete)
            self._watch_recipe_thread.start()
            self._watch_optimization_thread.start()
        else:
            raise Exception('template file is missing')
        
    def construct_optimiser_from_config_file(self, config_dict: dict):
        self._optimization_model = OptimizationState.from_dict(config_dict['optimizer'])
        self._max_recipe_count = self.recipe_count()
        self._optimizer = OptimizerBase(opt_module=self._optimization_model.optimizer_module,
                                        opt_class=self._optimization_model.optimizer_class, opt_hyperparameters=self._optimization_model.optimizer_hyperparameters)

    def recipe_count(self):
        return self._optimization_model.max_recipe_count


    def is_recipe_template_available(self):
        _template_path = Path(self._template_dir)
        return _template_path.is_file()

    # -------------optimisation related functions to be moved to optimisation base after confirmation------------#

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
        module = importlib.import_module(
            self._optimization_model.optimizer_module)
        optimizer_class = getattr(
            module, self._optimization_model.optimizer_class)
        return optimizer_class(self._optimization_model.optimizer_hyperparameters)

    def _pandas_to_params_targets(self, data: pd.DataFrame):
        """
        Split experiment result dataframe into dictionary format params and target array.
        """
        # get optimization target name
        if self.target_name is None:
            target_name = [
                i for i in data.columns if i not in self._component_keys]
            assert len(
                target_name) == 1, 'Require only one unseen column as optimization target'
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
    
    def update_model(self, data: pd.DataFrame, **kwargs):
        """
        Record probed points and update optimization model.
        :param data:

        """
        self._probed_points += data
        params, targets = self._pandas_to_params_targets(data)
        for param, target in zip(params, targets):
            self.model.register(param, target)