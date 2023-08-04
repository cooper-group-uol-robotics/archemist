import pandas as pd
from archemist.core.optimisation.optimiser_base import OptimiserBase
import random
from bayes_opt.bayesian_optimization import BayesianOptimization
import warnings
import numpy as np
from scipy.optimize import minimize



class BayesOptOptimiser(OptimiserBase):

    def __init__(self, **kwargs):
        """

        :param config_file:
            Config file for experiment and optimizer settings.
        :param batch_size:
            Number of probe points for each iteration during optimization.
        """
        optim_records = kwargs["optimization_records"]
        
        self._hyperparameters = optim_records.optimiser_args["hyperparameters"]
        self._batch_size = optim_records.optimiser_args['batch_size']
        self._components = optim_records.optimiser_args['components']
        self._target_name = optim_records.optimiser_args['target'] if 'target' in optim_records.optimiser_args else None
        if 'constraint' in optim_records.optimiser_args:
            self._constraint = {'type': optim_records.optimiser_args['constraint']['type'],
                            'fun': optim_records.optimiser_args['constraint']['expression']}
        else:
            self._constraint = None

        self._probed_points = []
        bound = self._generate_bound_from_config()
        self.component_keys = bound.keys()  # optimization component names
        
        stored_model = optim_records.stored_optimisation_obj
        if stored_model is not None:
            self.model = stored_model
        else:
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
        if self._constraint:
            return ConstrainedBayesianOptimization(constraints=self._constraint, **self._hyperparameters, **kwargs)
        else:
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
    
class ConstrainedBayesianOptimization(BayesianOptimization):

    def __init__(self, constraints, **kwargs):
        self.constraints = constraints
        super().__init__(**kwargs)

    def suggest(self, utility_function):
        if len(self._space) == 0:
            return self._space.array_to_params(self._space.random_sample())

        # Sklearn's GP throws a large number of warnings at times, but
        # we don't really need to see them here.
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self._gp.fit(self._space.params, self._space.target)

        # Finding argmax of the acquisition function.
        constraints=self.constraints
        constraints["fun"] = eval(constraints["fun"]) if isinstance(constraints["fun"], str) else constraints["fun"]
        suggestion = acq_max(
            ac=utility_function.utility,
            gp=self._gp,
            y_max=self._space.target.max(),
            bounds=self._space.bounds,
            random_state=self._random_state,
            constraints=self.constraints
        )

        return self._space.array_to_params(suggestion)


def acq_max(ac, gp, y_max, bounds, random_state, n_warmup=10000, n_iter=10, constraints=None):
    """
    A function to find the maximum of the acquisition function

    It uses a combination of random sampling (cheap) and the 'L-BFGS-B'
    optimization method. First by sampling `n_warmup` (1e5) points at random,
    and then running L-BFGS-B from `n_iter` (250) random starting points.

    Parameters
    ----------
    :param ac:
        The acquisition function object that return its point-wise value.

    :param gp:
        A gaussian process fitted to the relevant data.

    :param y_max:
        The current maximum known value of the target function.

    :param bounds:
        The variables bounds to limit the search of the acq max.

    :param random_state:
        instance of np.RandomState random number generator

    :param n_warmup:
        number of times to randomly sample the aquisition function

    :param n_iter:
        number of times to run scipy.minimize

    :param constraints:
        dictionary-format constraints for constraint bayesian optimization

    Returns
    -------
    :return: x_max, The arg max of the acquisition function.
    """

    # Warm up with random points
    x_tries = random_state.uniform(bounds[:, 0], bounds[:, 1],
                                size=(n_warmup, bounds.shape[0]))
    ys = ac(x_tries, gp=gp, y_max=y_max)
    x_max = x_tries[ys.argmax()]
    max_acq = ys.max()

    # Explore the parameter space more throughly
    x_seeds = random_state.uniform(bounds[:, 0], bounds[:, 1],
                                size=(n_iter, bounds.shape[0]))
    
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")

        for x_try in x_seeds:
            # Find the minimum of minus the acquisition function
            
            if constraints is not None:
                res = minimize(lambda x: -ac(x.reshape(1, -1), gp=gp, y_max=y_max),
                            x_try,  # x_try.reshape(1, -1),
                            bounds=bounds,
                            method="trust-constr",
                            constraints=constraints)

            else:
                res = minimize(lambda x: -ac(x.reshape(1, -1), gp=gp, y_max=y_max),
                            x_try.reshape(1, -1),
                            bounds=bounds,
                            method="L-BFGS-B")

            # See if success
            if not res.success:
                continue

            # Store it if better than previous minimum(maximum).
            if max_acq is None or -res.fun >= max_acq:
                x_max = res.x
                max_acq = -res.fun

    # Clip output to make sure it lies within the bounds. Due to floating
    # point technicalities this is not always the case.
    return np.clip(x_max, bounds[:, 0], bounds[:, 1])