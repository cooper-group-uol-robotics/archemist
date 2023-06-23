import pandas as pd
import importlib
from optimiser_base import OptimizerBase


class BayesOptOptimizer(OptimizerBase):

    def __init__(self,
                 config_file: str,
                #  recipe_dir: str,
                #  template_dir: str,
                 batch_size: int = 15,
                 ):
        """

        :param config_file:
            Config file for experiment and optimizer settings.
        :param batch_size:
            Number of probe points for each iteration during optimization.
        """
        super(BayesOptOptimizer, self).__init__(config_file=config_file,
                                                # recipe_dir=recipe_dir,
                                                # template_dir=template_dir,
                                                batch_size=batch_size)

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
        if 'optimizer' in self._config_dict:
            err_msg = 'Wrong module or class config provided, please check your config file'
            assert self._config_dict['optimizer']['module'] == 'bayes_opt.bayesian_optimization', err_msg
            assert self._config_dict['optimizer']['class'] == 'BayesianOptimization', err_msg
            super(BayesOptOptimizer, self).generate_model(**kwargs)
        else:
            from bayes_opt.bayesian_optimization import BayesianOptimization
            return BayesianOptimization(**kwargs)

    def update_model(self, data: pd.DataFrame, **kwargs):
        """
        Record probed points and update optimization model.
        :param data:

        """
        self._probed_points = self._probed_points.append(data)
        params, targets = self._pandas_to_params_targets(data)
        for param, target in zip(params, targets):
            self.model.register(param, target)

    def _pandas_to_params_targets(self, data: pd.DataFrame):
        """
        Split experiment result dataframe into dictionary format params and target array.
        """
        # get optimization target name
        if self.target_name is None:
            target_name = [i for i in data.columns if i not in self.component_keys]
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

    def generate_batch(self, **kwargs) -> pd.DataFrame:
        """
        Randomly generate optimization batch.
        Will change to other sampling strategy afterwards.
        """

        acquisition_function = self._generate_acquisition(**kwargs)

        batch = pd.DataFrame([])
        for _ in range(self.batch_size):
            x_probe = self.model.suggest(acquisition_function)
            batch = batch.append(x_probe, ignore_index=True)
        return batch

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


test_data = pd.DataFrame([[3,4], [1, 2]], columns=['x', 'y'])

opt = BayesOptOptimizer('/home/satheesh/ARChemeist_ws/src/archemist/tests/optimisation_test/optimization_config.yaml')
opt.update_model(test_data)