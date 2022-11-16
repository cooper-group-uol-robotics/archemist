from bson.objectid import ObjectId
from archemist.core.models.recipe_model import RecipeModel, StateDetails
from archemist.core.persistence.object_factory import StationFactory
from transitions import Machine

class Recipe:
    def __init__(self, recipe_model: RecipeModel):
        self._model = recipe_model
        self._station_sm = Machine(states=self._model.states,initial=self._model.current_state, transitions=self._model.transitions)
        
    @classmethod
    def from_dict(cls, recipe_document: dict):
        model = RecipeModel()
        model.name = recipe_document['general']['name']
        model.exp_id = recipe_document['general']['id']
        if 'materials' in recipe_document:
            if 'solids' in recipe_document['materials']:
                model.solids = [liquid_dict for liquid_dict in recipe_document['materials']['solids']]
            if 'liquids' in recipe_document['materials']:
                model.liquids = [liquid_dict for liquid_dict in recipe_document['materials']['liquids']]
        for state_dict in recipe_document['process']:
            model.states.append(state_dict['state_name'])
            model.transitions.extend([{'trigger': trigger, 'source': state_dict['state_name'],
                                'dest': state_dict['transitions'][trigger]} 
                                 for trigger in ['on_success','on_fail']])
            state_detail = StateDetails(station_type=state_dict['station']['type'],
                                station_id=int(state_dict['station']['id']),
                                station_op=StationFactory.create_op_from_dict(
                                            state_dict['station']['operation']).model)
            model.state_map.update({state_dict['state_name']:state_detail})
        model.states.append('end_state')
        model.current_state = model.states[0]
        model.save()
        return cls(model)

    @classmethod
    def from_object_id(cls, object_id: ObjectId):
        model = RecipeModel.objects.get(id=object_id)
        return cls(model)
        
    @property
    def name(self):
        return self._model.name

    @property
    def id(self):
        return self._model.exp_id

    @property
    def solids (self):
        self._model.reload('solids')
        return self._model.solids

    @property
    def liquids (self):
        self._model.reload('liquids')
        return self._model.liquids

    @property
    def model(self):
        self._model.reload()
        return self._model

    @property
    def current_state(self):
        self._model.reload('current_state')
        return self._model.current_state

    def advance_state(self, success: bool):
        self._update_recipe_sm_state()
        self._station_sm.on_success() if success else self._station_sm.on_fail()
        self._model.update(current_state=self._station_sm.state)
        self._logRecipe('Current state advanced to ' + self._station_sm.state)

    def is_complete(self):
        return self.current_state == 'end_state'

    def get_current_task_op(self):
        if not self.is_complete():
            self._update_recipe_sm_state()
            state_details = self._model.state_map[self.current_state]
            return StationFactory.create_op_from_model(state_details.station_op)

    def get_current_station(self):
        if not self.is_complete():
            self._update_recipe_sm_state()
            state_details = self._model.state_map[self.current_state]
            return state_details.station_type, state_details.station_id

    def get_next_station(self, success: bool):
        self._update_recipe_sm_state()
        self._station_sm.on_success() if success else self._station_sm.on_fail()        
        if self._station_sm.state != 'end_state':
            state_details = self._model.state_map[self._station_sm.state]
            return state_details.station_type, state_details.station_id
        else:
            return 'end',None

    def _update_recipe_sm_state(self):
        self._station_sm.state = self.current_state # update state machine state to be inline with db
        
    def _logRecipe(self, message: str):
        print(f'Recipe [{self.id}]: ' + message)


