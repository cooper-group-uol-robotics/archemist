from bson.objectid import ObjectId
from pymodm import MongoModel, fields
from archemist.persistence.dbObjProxy import DbObjProxy
from transitions import Machine

class RecipeModel(MongoModel):
    name = fields.CharField()
    id = fields.IntegerField()
    station_op_descriptors = fields.ListField(fields.DictField())
    solids = fields.ListField(fields.CharField())
    liquids = fields.ListField(fields.CharField())
    states = fields.ListField(fields.CharField())
    transitions = fields.ListField(fields.DictField())
    current_state = fields.CharField()

    class Meta:
        collection_name = 'Recipes'
        connection_alias = 'archemist_connection'

class Recipe:
    def __init__(self, recipe_model: RecipeModel):
        self._model = recipe_model
        self._station_sm = Machine(states=self._model.states,initial=self._model.current_state, transitions=self._model.transitions)
        
    @classmethod
    def from_dict(cls, recipe_document: dict):
        model = RecipeModel()
        model.name = recipe_document['name']
        model.id = recipe_document['id']
        model.station_op_descriptors = [stationOp for station_dict in recipe_document['stations'] for stationOp in station_dict['stationOps']]
        model.solids = recipe_document['materials']['solids']
        model.liquids = recipe_document['materials']['liquids']
        model.states = [state_dict['state_name'] for state_dict in recipe_document['workflowSM']]
        model.transitions = [{'trigger': trigger, 'source': state_dict['state_name'], 'dest': state_dict[trigger]} 
                        for state_dict in recipe_document['workflowSM'] for trigger in ['onSuccess','onFail']]
        model.current_state = recipe_document['current_state']
        model.save()
        return cls(model)

    @classmethod
    def from_object_id(cls, object_id: ObjectId):
        model = RecipeModel.objects.get({'_id': object_id})
        return cls(model)
        
    @property
    def name(self):
        return self._model.name

    @property
    def id(self):
        return self._model.id

    @property
    def solids (self):
        return self._model.solids

    @property
    def liquids (self):
        return self._model.liquids

    @property
    def model(self):
        return self._model

    @property
    def current_state(self):
        return self._model.current_state

    def advance_state(self, success: bool):
        self._update_recipe_sm_state()
        if success:
            self._station_sm.onSuccess()
        else:
            self._station_sm.onFail()
        self._model.current_state = self._station_sm.state
        # if self._station_sm.state == 'end':
        #     self._batch_db_proxy.update_field('processed', True)
        self._logRecipe('Current state advanced to ' + self._station_sm.state)

    def is_complete(self):
        return self._model.current_state == 'end'

    def get_current_task_op_dict(self):
        if not self.is_complete():
            self._update_recipe_sm_state()
            _,_, current_op_name = self._station_sm.state.split('.')
            current_op_dict = next(op_dict for op_dict in self._model.station_op_descriptors if op_dict['type'] == current_op_name)
            return current_op_dict

    def get_current_station(self):
        if not self.is_complete():
            self._update_recipe_sm_state()
            current_station_name,current_station_id, _ = self._station_sm.state.split('.')
            current_station_id = int(current_station_id.strip('id_'))
            return current_station_name, current_station_id

    def get_next_station(self, success: bool):
        self._update_recipe_sm_state()
        if success:
            self._station_sm.onSuccess()
        else:
            self._station_sm.onFail()
        
        if self._station_sm.state != 'end':
            next_station_name,next_station_id, _ = self._station_sm.state.split('.')
            next_station_id = int(next_station_id.strip('id_'))
            return next_station_name, next_station_id
        else:
            return 'end',None

    def _update_recipe_sm_state(self):
        self._station_sm.state = self._model.current_state # update state machine state to be inline with db
        
    def _logRecipe(self, message: str):
        print(f'Recipe [{self.id}]: ' + message)


