import pandas as pd
from archemist.core.optimisation.optimiser_base import OptimiserBase
import random
from bayes_opt.bayesian_optimization import BayesianOptimization

class BayesOptOptimiser(OptimiserBase):

    def __init__(self, args_dict: dict):
        """

        :param config_file:
            Config file for experiment and optimizer settings.
        :param batch_size:
            Number of probe points for each iteration during optimization.
        """
        self._hyperparameters = args_dict["hyperparameters"]
        self._batch_size = args_dict['batch_size']
        self._components = args_dict['components']
        self._target_name = args_dict['target'] if 'target' in args_dict else None

        self._probed_points = []
        bound = self._generate_bound_from_config()
        self.component_keys = bound.keys()  # optimization component names
        self.model = self._generate_model(f=None, pbounds=bound)

    def _generate_bound_from_config(self):
        """
        Generate optimization constraints for building model.
        This constraint format is for bayes_opt package.
        """
        constraint = {}
        for component, bounds in self._components.items():
            constraint[component] = (bounds['lower_bound'], bounds['upper_bound'])
        return constraint

    def _generate_model(self, **kwargs):
        """
        Generate model for optimization.
        """
        return BayesianOptimization(**self._hyperparameters, **kwargs)

    def update_model(self, data: pd.DataFrame, **kwargs) -> object:
        """
        Record probed points and update optimization model.
        Returns the updated model
        :param data:
        """
        self._probed_points.append(data)
        params, targets = self._pandas_to_params_targets(data)
        for param, target in zip(params, targets):
            self.model.register(param, target)
        return self.model

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
        for component, bounds in self._components.items():
            random_values[component] = []
            for element in range(self._batch_size):
                _val = random.uniform(bounds['lower_bound'], bounds['upper_bound'])
                _val = round(_val,2)
                random_values[component].append(_val)
        random_values = pd.DataFrame.from_dict(random_values)
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