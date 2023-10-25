from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.models.op_result_model import OpResultModel, MaterialOpResultModel, ProcessOpResultModel

from typing import Union, Type, Dict, Any
from bson.objectid import ObjectId

class OpResult:
    def __init__(self, result_model: Union[Type[OpResultModel], ModelProxy]):
        if isinstance(result_model, ModelProxy):
            self._model_proxy = result_model
        else:
            self._model_proxy = ModelProxy(result_model)

    @classmethod
    def _set_model_common_fields(cls, result_model: OpResultModel, origin_op_id: ObjectId):
        result_model._type = cls.__name__
        result_model._module = cls.__module__
        result_model.origin_op = origin_op_id


    @classmethod
    def from_args(cls, **kwargs):
        model = OpResultModel()
        origin_op = kwargs.get("origin_op")
        cls._set_model_common_fields(model, origin_op)
        model.save()
        return cls(model)

    @property
    def origin_op(self) -> ObjectId:
        return self._model_proxy.origin_op
    
class MaterialOpResult(OpResult):
    def __init__(self, result_model: Union[MaterialOpResultModel, ModelProxy]):
        super().__init__(result_model)

    @classmethod
    def from_args(cls, **kwargs):
        model = MaterialOpResultModel()
        origin_op = kwargs.get("origin_op")
        model.material_name = kwargs.get("material_name")
        model.material_id = kwargs.get("material_id")
        model.amount = kwargs.get("amount")
        model.unit = kwargs.get("unit")
        cls._set_model_common_fields(model, origin_op)
        model.save()
        return cls(model)
    
    @property
    def material_name(self) -> str:
        return self._model_proxy.material_name
    
    @property
    def material_id(self) -> int:
        return self._model_proxy.material_id
    
    @property
    def amount(self) -> float:
        return self._model_proxy.amount
    
    @property
    def unit(self) -> str:
        return self._model_proxy.unit
    
class ProcessOpResult(OpResult):
    def __init__(self, result_model: Union[ProcessOpResultModel, ModelProxy]):
        super().__init__(result_model)

    @classmethod
    def from_args(cls, **kwargs):
        model = ProcessOpResultModel()
        origin_op = kwargs.get("origin_op")
        model.parameters = kwargs.get("parameters")
        cls._set_model_common_fields(model, origin_op)
        model.save()
        return cls(model)
    
    @property
    def parameters(self) -> Dict[str, Any]:
        return dict(self._model_proxy.parameters)

    
