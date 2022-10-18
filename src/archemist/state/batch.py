from datetime import datetime
from typing import Any, List
from mongoengine import Document, EmbeddedDocument, fields
from bson.objectid import ObjectId
from pickle import loads,dumps
from archemist.state.recipe import Recipe,RecipeModel
from archemist.util import Location

class SampleModel(EmbeddedDocument):
    rack_index = fields.IntField(min_value=0, required=True)
    materials = fields.ListField(fields.StringField(), default=[])
    capped = fields.BooleanField(default=False)
    operation_ops = fields.ListField(fields.BinaryField(), default=[])

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

class BatchModel(Document):
    exp_id = fields.IntField(required=True)
    num_samples = fields.IntField(min_value=1, required=True)
    location = fields.DictField()
    recipe = fields.ReferenceField(RecipeModel, null=True)
    samples = fields.EmbeddedDocumentListField(SampleModel)
    current_sample_index = fields.IntField(min_value=0, default=0)
    station_history = fields.ListField(fields.StringField(), default=[])

    meta = {'collection': 'batches', 'db_alias': 'archemist_state'}

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

    def attach_recipe(self, recipe_dict: dict):
        if isinstance(recipe_dict, dict):
            recipe_dict['current_state'] = 'start'
            new_recipe = Recipe.from_dict(recipe_dict).model
            self._model.update(recipe=new_recipe)
        else:
            raise ValueError

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
        self._model.reload('samples')
        current_index = self.current_sample_index
        return Sample(self._model.samples[current_index])

    def get_samples_list(self) -> List[Sample]:
        self._model.reload('samples')
        return [Sample(model) for model in self._model.samples]

    def process_current_sample(self):
        self._model.update(inc__current_sample_index=1)
        if (self.current_sample_index == self.num_samples):
            self._model.update(current_sample_index=0)
            self._log_batch('All samples have been processed. Batch index is reset to 0.')

    def add_station_op_to_current_sample(self, station_op: Any):
        current_sample = self.get_current_sample()
        current_sample.add_operation_op(station_op)
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

    def _log_batch(self, message: str):
        print(f'[{self}]: {message}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}-{self.id}'

