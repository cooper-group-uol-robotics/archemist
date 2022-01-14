from datetime import datetime

from bson.objectid import ObjectId
from archemist.persistence.dbObjProxy import DbObjProxy
from archemist.state.recipe import Recipe
from archemist.util import Location

class Sample():
    def __init__(self, sample_dict: dict):
        self._rack_indx = sample_dict['rack_index']
        self._materials = sample_dict['materials']
        self._capped = sample_dict['capped']
        self._operation_ops = [DbObjProxy.decode_object(encoded_op) for encoded_op in sample_dict['operation_ops']]

    @classmethod
    def from_index(cls, rack_index: int):
        return cls({'rack_index': rack_index, 'materials': [], 'capped': False, 'operation_ops': []})

    @classmethod
    def from_dict(cls, sample_dict: dict):
        return cls(sample_dict)

    @property
    def rack_index(self):
        return self._rack_indx

    @property
    def materials(self):
        return self._materials

    @property
    def capped(self):
        return self._capped

    @property
    def operation_ops(self):
        return self._operation_ops

    def to_dict(self):
        return {
            'rack_index': self._rack_indx,
            'materials': self._materials,
            'capped': self._capped,
            'operation_ops': self._operation_ops
        }


class Batch(DbObjProxy):

    def __init__(self, **kwargs):
        if len(kwargs) > 2:
            batch_document = dict()
            batch_document['object'] = self.__class__.__name__
            batch_document['id'] = kwargs['batch_id']
            
            recipe_dict = kwargs['recipe_dict']
            recipe_dict['current_state'] = 'start'
            batch_document['recipe'] = recipe_dict

            batch_document['location'] = kwargs['location'].to_dict()
            
            batch_document['samples'] = []
            for indx in range(0,kwargs['num_samples']):
                sample = Sample.from_index(indx)
                batch_document['samples'].append(sample.to_dict())

            batch_document['num_samples'] = kwargs['num_samples']
            batch_document['current_sample_index'] = 0
            batch_document['all_samples_processed'] = False
            batch_document['station_history'] = []

            super().__init__(kwargs['db'], 'batches', batch_document)
        else:
            super().__init__(kwargs['db'], 'batches', kwargs['object_id'])
        
        self._recipe = Recipe(self.get_field('recipe'), self.get_db_proxy())

    @classmethod
    def from_arguments(cls, db: str, batch_id: int, recipe_dict: dict, num_samples: int, location:Location):
        return cls(db=db, batch_id=batch_id, recipe_dict=recipe_dict, num_samples=num_samples, location=location)

    @classmethod
    def from_objectId(cls, db: str, object_id: ObjectId):
        return cls(db=db, object_id=object_id)

    @property
    def id(self):
        return self.get_field('id')

    @property
    def recipe(self):
        return self._recipe

    @property
    def location(self):
        loc_dict = self.get_field('location')
        return Location(node_id=loc_dict['node_id'],graph_id=loc_dict['graph_id'], frame_name=loc_dict['frame_name'])

    @location.setter
    def location(self, location):
        if isinstance(location, Location):
            self.update_field('location', location.to_dict())
        else:
            raise ValueError

    # @property
    # def assigned(self):
    #     return self._assigned

    # @assigned.setter
    # def assigned(self, value):
    #     if isinstance(value, bool):
    #         self._assigned = value
    #     else:
    #         raise ValueError

    @property
    def current_sample_index(self):
        return self.get_field('current_sample_index')

    def get_samples_list(self):
        samples_dicts = self.get_field('samples')
        samples_list = list()
        for sample_dict in samples_dicts:
            samples_list.append(Sample.from_dict(sample_dict))
        
        return samples_list


    def process_current_sample(self):
        self.increment_field('current_sample_index')
        if (self.current_sample_index == (self.get_field('num_samples'))):
            self.update_field('all_samples_processed', True)
            self.update_field('current_sample_index', 0)
            self._log_batch('All samples have been processed. Batch index is reset to 0.')

    def add_station_op_to_current_sample(self, station_op):
        self.push_to_array_field(f'samples.{self.current_sample_index}.operation_ops', self.encode_object(station_op))

    def add_material_to_current_sample(self, material):
        self.push_to_array_field(f'samples.{self.current_sample_index}.materials', material)

    @property
    def station_history(self):
        return self.get_field('station_history')

    def add_station_stamp(self, station_stamp: str):
        self.push_to_array_field('station_history',f'{datetime.now()} , {station_stamp}')
        self._log_batch(f'({station_stamp}) stamp is added.')

    def are_all_samples_processed(self):
        return self.get_field('all_samples_processed')

    def reset_samples_processing(self):
        return self.update_field('all_samples_processed', False)


    @property
    def num_samples(self):
        return self.get_field('num_samples')

    def _log_batch(self, message: str):
        print(f'[{self}]: {message}')

    def __str__(self) -> str:
        return f'{self.__class__.__name__}-{self.id}'

