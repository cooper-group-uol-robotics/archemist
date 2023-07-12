import pandas as pd
import importlib
from optimiser_base import OptimizerBase
import random
from optimisation_state import OptimizationState
from object_factory import OptimizationFactory
from pathlib import Path
from archemist.core.persistence.yaml_handler import YamlHandler




class BayesOptOptimizer(OptimizerBase):

    def __init__(self,
                #  optimization_model,
                 config_dict: dict,
                 ):
        """

        :param config_file:
            Config file for experiment and optimizer settings.
        :param batch_size:
            Number of probe points for each iteration during optimization.
        """
        # self._optimization_model = optimization_model
        self._optimization_model = OptimizationFactory.create_from_dict(config_dict['optimizer'])
        self._config_dict = config_dict
        self._probed_points = []
        self._target_name = None
        self._batch_size = self._config_dict['optimizer']['batch_size']

        bound = self._generate_bound_from_config()
        self.component_keys = bound.keys()  # optimization component names
        self.model = self.generate_model(f=None, pbounds=bound)

    def _generate_bound_from_config(self):
        """
        Generate optimization constraints for building model.
        This constraint format is for bayes_opt package.
        """
        constraint = {}
        for key in self._config_dict['experiment']['components']:
            constraint[key] = (self._config_dict['experiment']['components'][key]['lower_bound'],
                               self._config_dict['experiment']['components'][key]['upper_bound'])
        return constraint

    def generate_model(self, **kwargs):
        """
        Generate model for optimization.
        """
        module = importlib.import_module(self._optimization_model.optimizer_module)
        optimizer_class = getattr(module, self._optimization_model.optimizer_class)
        return optimizer_class(**self._optimization_model.optimizer_hyperparameters, **kwargs)

    def update_model(self, data: pd.DataFrame, **kwargs):
        """
        Record probed points and update optimization model.
        :param data:
        """
        self._probed_points.append(data)
        params, targets = self._pandas_to_params_targets(data)
        for param, target in zip(params, targets):
            self.model.register(param, target)

    def _pandas_to_params_targets(self, data: pd.DataFrame):
        """
        Split experiment result dataframe into dictionary format params and target array.
        """
        # get optimization target name
        if self._target_name is None:
            target_name = [i for i in data.columns if i not in self.component_keys]
            assert len(target_name) == 1, 'Require only one unseen column as optimization target'
            self._target_name = target_name[0]
        else:
            assert self._target_name in data.columns, 'Optimization target not in result data, please check dataframe ' \
                                                     'format.'

        targets = data[self._target_name].to_numpy()
        data = data.drop(columns=self._target_name)
        params = []
        for row_index in range(len(data)):
            params.append(data.iloc[row_index].to_dict())
        return params, targets

    def generate_batch(self, **kwargs) -> pd.DataFrame:
        """
        Randomly generate optimization batch.
        Will change to other sampling strategy afterwards.
        """

        acquisition_function = self._generate_acquisition(**kwargs)

        batch = []
        for _ in range(self._batch_size):
            x_probe = self.model.suggest(acquisition_function)
            batch.append(x_probe)
        batch = pd.DataFrame.from_dict(batch)
        return batch

    def generate_random_values(self):
        # fenerate random pd frame from config file
        random_values = {}
        for key in self._config_dict['experiment']['components']:
            random_values[key] = []
            for element in range(self._batch_size):
                _val = random.uniform(self._config_dict['experiment']['components'][key]['lower_bound'],
                               self._config_dict['experiment']['components'][key]['upper_bound'])
                _val = round(_val,2)
                random_values[key].append(_val)
        return random_values

    @staticmethod
    def _generate_acquisition(**kwargs):
        """
        Generate acquisition function for BO.
        Default setting is Upper Confidence Bound with kappa=2.576.
        """
        from bayes_opt.util import UtilityFunction
        return UtilityFunction(
            kind=kwargs.get('acq', 'ucb'),
            kappa=kwargs.get('kappa', 2.576),
            xi=kwargs.get('xi', 0.0),
            kappa_decay=kwargs.get('kappa_decay', 1),
            kappa_decay_delay=kwargs.get('kappa_decay_delay', 0)
        )


# test_data = pd.DataFrame([[0.3,0.4,21, 10], [0.1, 0.2, 43.3, 1]], columns=['dye_A', 'dye_B', 'water', 'light_intensity'])
# print(test_data)
# cwd_path = Path.cwd()
# print(cwd_path)
# workflow_dir = Path.joinpath(cwd_path, "tests/optimisation_test")
# _config_file = Path.joinpath(workflow_dir, "config_files/optimization_config.yaml")
# _config_dict = YamlHandler.loadYamlFile(_config_file)

# opt = BayesOptOptimizer(_config_dict)
# opt.update_model(test_data)
# batch = opt.generate_batch()
# print(batch)
