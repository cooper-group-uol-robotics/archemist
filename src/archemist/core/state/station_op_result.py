from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.models.op_result_model import OpResultModel, MaterialsOpResultModel, ProcessOpResultModel

from typing import Union, Type, Dict, Any, List
from bson.objectid import ObjectId

class StationOpResult:
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
    def from_args(cls, origin_op: ObjectId):
        model = OpResultModel()
        origin_op = origin_op#kwargs.get("origin_op")
        cls._set_model_common_fields(model, origin_op)
        model.save()
        return cls(model)

    @property
    def model(self) -> Type[OpResultModel]:
        return self._model_proxy.model
    
    @property
    def object_id(self) -> ObjectId:
        return self._model_proxy.object_id

    @property
    def origin_op(self) -> ObjectId:
        return self._model_proxy.origin_op
    
    def __eq__(self, __value: object) -> bool:
        return self.object_id == __value.object_id
    
class MaterialOpResult(StationOpResult):
    def __init__(self, result_model: Union[MaterialsOpResultModel, ModelProxy]):
        super().__init__(result_model)

    @classmethod
    def from_args(cls, origin_op: ObjectId, material_names: List[str],
                  amounts: List[float],
                  units: List[str]):
        model = MaterialsOpResultModel()
        origin_op = origin_op
        cls._set_model_common_fields(model, origin_op)
        model.material_names = material_names
        model.amounts = amounts
        model.units = units
        model.save()
        return cls(model)
    
    @property
    def material_names(self) -> List[str]:
        return self._model_proxy.material_names
    
    @property
    def amounts(self) -> List[float]:
        return self._model_proxy.amounts
    
    @property
    def units(self) -> List[str]:
        return self._model_proxy.units
    
class ProcessOpResult(StationOpResult):
    def __init__(self, result_model: Union[ProcessOpResultModel, ModelProxy]):
        super().__init__(result_model)

    @classmethod
    def from_args(cls, origin_op: ObjectId, parameters: Dict[str, Any]):
        model = ProcessOpResultModel()
        origin_op = origin_op
        cls._set_model_common_fields(model, origin_op)
        model.parameters = parameters
        model.save()
        return cls(model)
    
    @property
    def parameters(self) -> Dict[str, Any]:
        return dict(self._model_proxy.parameters)

    
