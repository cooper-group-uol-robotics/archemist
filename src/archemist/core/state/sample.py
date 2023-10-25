from archemist.core.models.sample_model import SampleModel
from archemist.core.persistence.object_factory import OpResultFactory
from archemist.core.persistence.models_proxy import ModelProxy, ListProxy
from archemist.core.state.op_result import OpResult, MaterialOpResult

from bson.objectid import ObjectId
from typing import Any, List, Union, Dict, Type

class Sample:
    def __init__(self, sample_model: Union[SampleModel, ModelProxy]):
        if isinstance(sample_model, ModelProxy):
            self._model_proxy = sample_model
        else:
            self._model_proxy = ModelProxy(sample_model)

    @classmethod
    def from_args(cls, parent_batch_id: ObjectId):
        model = SampleModel()
        model.parent_batch_id = parent_batch_id
        model.save()
        return cls(model)
    
    @classmethod
    def from_object_id(cls, object_id: ObjectId):
        model = SampleModel.objects.get(id=object_id)
        return cls(model)
    
    @property
    def model(self) -> SampleModel:
        return self._model_proxy.model
    
    @property
    def object_id(self) -> ObjectId:
        return self._model_proxy.object_id
    
    @property
    def parent_batch_id(self) -> ObjectId:
        return self._model_proxy.parent_batch_id

    @property
    def materials(self) -> Dict[str, Any]:
        return self._model_proxy.materials
    
    @property
    def result_ops(self) -> List[Type[OpResult]]:
        return ListProxy(self._model_proxy.result_ops, OpResultFactory.create_from_model)

    def add_result_op(self, result_op: Type[OpResult]):
        if isinstance(result_op, MaterialOpResult):
            material_name = result_op.material_name
            if self.materials.get(material_name, None):
                new_material_details = self.materials[material_name]
                new_material_details["amount"] += result_op.amount #TODO handle units
                self.materials[material_name] = new_material_details
            else:
               material_details = {"amount": result_op.amount, "unit": result_op.unit}
               self.materials[material_name] = material_details
        self.result_ops.append(result_op)

    def __eq__(self, other_sample) -> bool:
        return self.object_id == other_sample.object_id

    
