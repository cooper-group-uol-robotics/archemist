import pandas as pd
import abc
import yaml
import importlib
from archemist.core.persistence.yaml_handler import YamlHandler


class OptimizerBase(abc.ABC):
    def __init__(self, **kwargs) -> None:
        self._module = kwargs['opt_module']
        self._class = kwargs['opt_class']
        self._class = kwargs['opt_hyperparameters']
        super().__init__()

    @abc.abstractmethod
    def generate_batch(self, **kwargs) -> pd.DataFrame:
        raise NotImplementedError

    @abc.abstractmethod
    def update_model(self, data: pd.DataFrame, **kwargs):
        raise NotImplementedError

    @abc.abstractmethod
    def generate_model(self, **kwargs):
        raise NotImplementedError

