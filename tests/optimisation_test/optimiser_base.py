import pandas as pd
import abc
import yaml
import importlib
from archemist.core.persistence.yaml_handler import YamlHandler


class OptimizerBase(abc.ABC):

    @abc.abstractmethod
    def generate_batch(self, **kwargs) -> pd.DataFrame:
        raise NotImplementedError

    @abc.abstractmethod
    def update_model(self, data: pd.DataFrame, **kwargs):
        raise NotImplementedError

    @abc.abstractmethod
    def generate_model(self, **kwargs):
        raise NotImplementedError

    @staticmethod
    def _read_config(file_path: str):
        """
        Read config file for experiment setting and optimization algorithm hyper-parameter settings.
        Config file should be yaml (.yml) file.

        :param file_path:
            Config .yml file directory.
        """
        return YamlHandler.load_config_file(file_path)
        # with open(file_path, 'r') as yaml_file:
        #     return yaml.load(yaml_file, Loader=yaml.FullLoader)