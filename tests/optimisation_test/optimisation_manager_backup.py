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
        self.component_keys = bound.keys()  # optimization component names
        self.target_name = None  # optimization target name, will update when receiving result dataframe

        self.model = self.generate_model(pbounds=bound)
        self._probed_points = pd.DataFrame([])
    
    def construct_optimiser_from_config_file(self, config_file_path:str):
        return State.from_dict(self.config['optimizer'])

    def generate_bound_from_config(self):
        """
        Generate optimization constraints for building model.
        This constraint format is for bayes_opt package.
        """
        constraint = {}
        for key in self.config['experiment']['components']:
            constraint[key] = (self.config['experiment']['components'][key]['lower_bound'],
                               self.config['experiment']['components'][key]['upper_bound'])
        return constraint

    ##################
    def generate_model(self, **kwargs):
        """
        Generate optimization model from config file based on the 'optimizer' part.
        If the model requires constraint information, please pass it via kwargs.
        """
        module = importlib.import_module(self.config['optimizer']['module'])
        optimizer_class = getattr(module, self.config['optimizer']['class'])
        return optimizer_class(**self.config['optimizer']['hyperparameters'], **kwargs)

    def update_model(self, data: pd.DataFrame, **kwargs):
        """
        Record probed points and update optimization model.
        :param data:

        """
        self._probed_points = self._probed_points.append(data)
        params, targets = self.pandas_to_params_targets(data)
        for param, target in zip(params, targets):
            self.model.register(param, target)

    def pandas_to_params_targets(self, data: pd.DataFrame):
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
        pass
        # acquisition_function = self.generate_acquisition(**kwargs)

        # batch = pd.DataFrame([])
        # for _ in range(self.batch_size):
        #     x_probe = self.model.suggest(acquisition_function)
        #     batch = batch.append(x_probe, ignore_index=True)
        # return batch

    
    @staticmethod
    def generate_acquisition(**kwargs):

        pass
        """
        Generate acquisition function for BO.
        Default setting is Upper Confidence Bound with kappa=2.576.
        """
        # from bayes_opt.util import UtilityFunction
        # return UtilityFunction(
        #     kind=kwargs.get('acq', 'ucb'),
        #     kappa=kwargs.get('kappa', 2.576),
        #     xi=kwargs.get('xi', 0.0),
        #     kappa_decay=kwargs.get('kappa_decay', 1),
        #     kappa_decay_delay=kwargs.get('kappa_decay_delay', 0)
        # )
    
###########################

    def is_recipe_template_available(self, _template_path):
        _template_path = Path(_template_path)
        return _template_path.is_file()
