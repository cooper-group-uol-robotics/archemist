import pandas as pd
import abc


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

    @abc.abstractmethod
    def _generate_model(self, **kwargs):
        """
        Generate optimization model from config file based on the 'optimizer' part.
        If the model requires constraint information, please pass it via kwargs.
        Overwrite this method to manually generate optimization model. If this method is not overwritten, module
        information must be passed through config file.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def _generate_bound_from_config(self, **kwargs):
        raise NotImplementedError
