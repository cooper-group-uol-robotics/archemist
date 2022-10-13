import importlib
from typing import Any

class ObjectFactory:

    @classmethod
    def construct_station_op_from_model(cls, op_model: Any):
        module = importlib.import_module(op_model._module)
        cls = getattr(module,op_model._type)
        return cls(op_model)

    @classmethod
    def construct_robot_op_from_model(cls, op_model: Any):
        module = importlib.import_module(op_model._module)
        cls = getattr(module,op_model._type)
        return cls(op_model)