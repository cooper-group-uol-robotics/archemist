import pandas as pd
import abc
from optimisation_state import OptimizationState
from pathlib import Path
from threading import Thread
from archemist.core.persistence.yaml_handler import YamlHandler
#from optimisation_handler import OptimizationHandler
import importlib


class OptimizerBase(abc.ABC):
    def __init__(self,
                 **kwargs) -> None:

        self.kwargs = kwargs

    @abc.abstractmethod
    def generate_batch(self, **kwargs) -> pd.DataFrame:
        raise NotImplementedError

    @abc.abstractmethod
    def update_model(self, data: pd.DataFrame, **kwargs):
        raise NotImplementedError

    def generate_model(self, **kwargs):
        """
        Generate optimization model from config file based on the 'optimizer' part.
        If the model requires constraint information, please pass it via kwargs.
        Overwrite this method to manually generate optimization model. If this method is not overwritten, module
        information must be passed through config file.
        """
        module = importlib.import_module(self._config_dict['optimizer']['module'])
        optimizer_class = getattr(module, self._config_dict['optimizer']['class'])
        return optimizer_class(**self._config_dict['optimizer']['hyperparameters'], **kwargs)

    @abc.abstractmethod
    def _generate_bound_from_config(self, **kwargs):
        raise NotImplementedError
