from datetime import datetime
from typing import Any, List
from bson.objectid import ObjectId
from pickle import loads,dumps
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.state.recipe import Recipe
from archemist.core.models.batch_model import SampleModel,BatchModel
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.util import Location

class Sample:
    def __init__(self, sample_model: SampleModel):
        self._model = sample_model

    @property
    def model(self):
        return self._model

    @property
    def rack_index(self):
        return self._model.rack_index

    @property
    def materials(self):
        return self._model.materials

    @property
    def capped(self):
        return self._model.capped

    @capped.setter
    def capped(self, capped: bool):
        self._model.capped = capped

    @property
    def operation_ops(self):
        return [StationFactory.create_op_from_model(op_model) for op_model in self._model.operation_ops]

    def add_operation_op(self, operation_op: Any):
        self._model.operation_ops.append(operation_op)

    def add_material(self, material: str):
        self._model.materials.append(material)

    def extract_op_data(self, op_field_dict: dict) -> dict:
        out_data = {}
        for op_type, fields in op_field_dict.items():
            op_instances = 0 # for duplicate ops
            for op in self.operation_ops:
                if op_type == op.type:
                    instance_suffix = "" if op_instances == 0 else f"_{op_instances}"
                    for field in fields:
                        field_value = getattr(op, field)
                        if type(field_value) is not dict:
                            out_data[field+instance_suffix] = field_value
                            #out_data[field] = field_value
                        else:
                            out_data.update({sub_field+instance_suffix:val for sub_field,val in field_value.items()})
                            #out_data.update({sub_field:val for sub_field,val in field_value.items()})
                    op_instances += 1
        return out_data

class Batch:
    def __init__(self, batch_model: BatchModel) -> None:
        self._model = batch_model

    @classmethod
    def from_arguments(cls, batch_id: int, num_samples: int, location:Location):
        model = BatchModel()
        model.exp_id = batch_id
        model.num_samples = num_samples
        model.location = location.to_dict()
        for i in range(0,num_samples):
            model.samples.append(SampleModel(rack_index=i))
        model.save()
        return cls(model)

    @classmethod
    def from_object_id(cls, object_id: ObjectId):
        model = BatchModel.objects.get(id=object_id)
        return cls(model)

    @property
    def model(self) -> BatchModel:
        self._model.reload()
        return self._model

    @property
    def id(self) -> int:
        return self._model.exp_id

    @property
    def recipe_attached(self) -> bool:
        self._model.reload('recipe')
        return self._model.recipe is not None

    @property
    def recipe(self) -> Recipe:
        if self.recipe_attached:
            return Recipe(self._model.recipe)

    def attach_recipe(self, recipe: Recipe):
        self._model.update(recipe=recipe.model)

    @property
    def location(self) -> Location:
        self._model.reload('location')
        loc_dict = self._model.location
        return Location(node_id=loc_dict['node_id'],graph_id=loc_dict['graph_id'], frame_name=loc_dict['frame_name'])

    @location.setter
    def location(self, location):
        if isinstance(location, Location):
            self._model.update(location=location.to_dict())
        else:
            raise ValueError

    @property
    def num_samples(self) -> int:
        return self._model.num_samples

    @property
    def current_sample_index(self):
        self._model.reload('current_sample_index')
        return self._model.current_sample_index

    def get_current_sample(self) -> Sample:
        current_index = self.current_sample_index
        self._model.reload('samples')
        return Sample(self._model.samples[current_index])

    def get_samples_list(self) -> List[Sample]:
        self._model.reload('samples')
        return [Sample(model) for model in self._model.samples]

    def process_current_sample(self):
        self._model.update(inc__current_sample_index=1)
        if (self.current_sample_index == self.num_samples):
            self._model.update(current_sample_index=0)
            self._log_batch('All samples have been processed. Batch index is reset to 0.')

    def add_station_op_to_current_sample(self, station_op: StationOpDescriptor):
        current_sample = self.get_current_sample()
        current_sample.add_operation_op(station_op.model)
        self._model.update(**{f'samples__{self._model.current_sample_index}':current_sample.model}) #https://stackoverflow.com/questions/2932648/how-do-i-use-a-string-as-a-keyword-argument
        #self._model.update(samples=self._model.samples) alternatively but the whole list will be updated

    def add_material_to_current_sample(self, material: str):
        current_sample = self.get_current_sample()
        current_sample.add_material(material)
        self._model.update(**{f'samples__{self._model.current_sample_index}':current_sample.model})

    @property
    def station_history(self):
        self._model.reload('station_history')
        return self._model.station_history

    def add_station_stamp(self, station_stamp: str):
        timed_stamp = f'{datetime.now()} , {station_stamp}'
        self._model.update(push__station_history=timed_stamp)
        self._log_batch(f'({station_stamp}) stamp is added.')

    def extract_samples_op_data(self, op_field_dict: dict):
        out_data = {}
        samples = self.get_samples_list()
        for index,sample in enumerate(samples):
            sample_data = sample.extract_op_data(op_field_dict)
            for field, val in sample_data.items():
                if index == 0:
                    out_data[field]= [val]
                else:
                    out_data[field].append(val)
        return out_data

    def _log_batch(self, message: str):
        print(f'[{self}]: {message}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}-{self.id}'
    
    def __eq__(self, other_batch) -> bool:
        return self.model.exp_id == other_batch.model.exp_id

