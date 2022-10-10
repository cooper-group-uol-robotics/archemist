from datetime import datetime
from typing import Any, List
from pymodm import MongoModel, EmbeddedMongoModel, fields
from bson.objectid import ObjectId
from pickle import loads,dumps
from archemist.state.recipe import Recipe,RecipeModel
from archemist.util import Location

class SampleModel(EmbeddedMongoModel):
    rack_index = fields.IntegerField(min_value=0)
    materials = fields.ListField(fields.CharField())
    capped = fields.BooleanField()
    operation_ops = fields.ListField(fields.BinaryField())

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
        return [loads(encoded_op) for encoded_op in self._model.operation_ops]

    def add_operation_op(self, operation_op: Any):
        self._model.operation_ops.append(dumps(operation_op))

    def add_material(self, material: str):
        self._model.materials.append(material)

    def to_dict(self):
        return {
            'rack_index': self.rack_indx,
            'materials': self.materials,
            'capped': self.capped,
            'operation_ops': self.operation_ops
        }

class BatchModel(MongoModel):
    id = fields.IntegerField()
    num_samples = fields.IntegerField(min_value=1)
    location = fields.DictField()
    recipe = fields.ReferenceField(RecipeModel, blank=True)
    samples = fields.EmbeddedDocumentListField(SampleModel)
    current_sample_index = fields.IntegerField(min_value=0, default=0)
    station_history = fields.ListField(fields.CharField(), blank=True)

    class Meta:
        collection_name = 'batches'
        connection_alias = 'archemist_connection'


class Batch:
    def __init__(self, batch_model: BatchModel) -> None:
        self._model = batch_model

    @classmethod
    def from_arguments(cls, batch_id: int, num_samples: int, location:Location):
        model = BatchModel()
        model.id = batch_id
        model.num_samples = num_samples
        model.location = location.to_dict()
        for i in range(0,num_samples):
            model.samples.append(SampleModel(rack_index=i))
        model.save()
        return cls(model)

    @classmethod
    def from_object_id(cls, object_id: ObjectId):
        model = BatchModel.objects.get({'_id': object_id})
        return cls(model)

    @property
    def model(self) -> BatchModel:
        return self._model

    @property
    def id(self) -> int:
        return self._model.id

    @property
    def recipe_attached(self) -> bool:
        return self._model.recipe is not None

    @property
    def recipe(self) -> Recipe:
        if self._model.recipe is not None:
            return Recipe(self._model.recipe)

    def attach_recipe(self, recipe_dict: dict):
        if isinstance(recipe_dict, dict):
            recipe_dict['current_state'] = 'start'
            self._model.recipe = Recipe.from_dict(recipe_dict).model
            self._model.save()
        else:
            raise ValueError

    @property
    def location(self) -> Location:
        loc_dict = self._model.location
        return Location(node_id=loc_dict['node_id'],graph_id=loc_dict['graph_id'], frame_name=loc_dict['frame_name'])

    @location.setter
    def location(self, location):
        if isinstance(location, Location):
            self._model.location = location.to_dict()
            self._model.save()
        else:
            raise ValueError

    @property
    def num_samples(self) -> int:
        return self._model.num_samples

    def get_current_sample(self) -> Sample:
        samples = self.get_samples_list()
        current_index = self._model.current_sample_index
        return samples[current_index]

    def get_samples_list(self) -> List[Sample]:
        return [Sample(model) for model in self._model.samples]

    def process_current_sample(self):
        self._model.current_sample_index += 1
        if (self._model.current_sample_index == self.num_samples):
            self._model.current_sample_index = 0
            self._log_batch('All samples have been processed. Batch index is reset to 0.')
        self._model.save()

    def add_station_op_to_current_sample(self, station_op: Any):
        current_sample = self.get_current_sample()
        current_sample.add_operation_op(station_op)
        self._model.save()

    def add_material_to_current_sample(self, material: str):
        current_sample = self.get_current_sample()
        current_sample.add_material(material)
        self._model.save()

    @property
    def station_history(self):
        return self._model.station_history

    def add_station_stamp(self, station_stamp: str):
        timed_stamp = f'{datetime.now()} , {station_stamp}'
        self._model.station_history.append(timed_stamp)
        self._model.save()
        self._log_batch(f'({station_stamp}) stamp is added.')

    def _log_batch(self, message: str):
        print(f'[{self}]: {message}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}-{self.id}'

