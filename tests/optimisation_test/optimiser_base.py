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
                 config_file: str,
                #  recipe_dir: str,
                #  template_dir: str,
                 batch_size: int = 15,
                 **kwargs) -> None:

        self._config_dict = YamlHandler.loadYamlFile(config_file)
        # self._recipes_dir = recipe_dir
        # self._template_dir = template_dir  # todo: these two thing

        # save config to database
        self._database_model = OptimizationState.from_dict(self._config_dict['optimizer'])
        #self._max_recipe_count = self.recipe_count()

        self.batch_size = batch_size
        self.target_name ='y'  # optimization target name, will update when receiving result dataframe
        self._probed_points = pd.DataFrame([])

        self.kwargs = kwargs

    #     if self._is_recipe_template_available:
    #         self._handler = OptimizationHandler(self._recipes_dir, self._max_recipe_count,
    #                                             self._template_dir)  # todo: max_recipe_count
    #         self._watch_recipe_thread = Thread(target=self._handler.watch_recipe_queue)
    #         self._watch_optimization_thread = Thread(target=self._handler.watch_batch_complete)
    #         self._watch_recipe_thread.start()
    #         self._watch_optimization_thread.start()

    # def _is_recipe_template_available(self):
    #     _template_path = Path(self._template_dir)
    #     return _template_path.is_file()

    # def recipe_count(self):
    #     return self._database_model.max_recipe_count

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
