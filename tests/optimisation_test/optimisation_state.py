from optimisation_model import OptimisationModel


class OptimizationState:
    def __init__(self, optimization_model: OptimisationModel) -> None:
        self._model = optimization_model

    @classmethod
    def from_dict(cls, state_dict: dict = {}):
        model = OptimisationModel()
        model.optimizer_module = state_dict['module']
        model.optimizer_class = state_dict['class']
        model.optimizer_hyperparameters = state_dict['hyperparameters']
        model.max_recipe_count = state_dict['max_recipe_count']  # todo: what is this?
        model.save()
        return cls(model)

    @property
    def optimizer_module(self) -> str:
        return self._model.optimizer_module

    @property
    def optimizer_class(self) -> str:
        return self._model.optimizer_class

    @property
    def optimizer_hyperparameters(self):
        return self._model.optimizer_hyperparameters

    @property
    def max_recipe_count(self) -> int:
        return self._model.max_recipe_count