from typing import Union, List, Dict, Tuple
from bson.objectid import ObjectId
from mongoengine import Document
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.models.recipe_model import RecipeModel, StateDetails
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.persistence.models_proxy import ModelProxy
from transitions import Machine

class Recipe:
    def __init__(self, recipe_model: Union[RecipeModel, ModelProxy]):
        if isinstance(recipe_model, ModelProxy):
            self._model_proxy = recipe_model
        else:
            self._model_proxy = ModelProxy(recipe_model)
        self._station_sm = Machine(states=list(self._model_proxy.states),initial=self._model_proxy.current_state,
                                   transitions=list(self._model_proxy.transitions))
        
    @classmethod
    def from_dict(cls, recipe_dict: dict):
        model = RecipeModel()
        model.name = recipe_dict['general']['name']
        model.exp_id = recipe_dict['general']['id']
        if 'materials' in recipe_dict:
            if 'solids' in recipe_dict['materials']:
                model.solids = [solid_dict for solid_dict in recipe_dict['materials']['solids']]
            if 'liquids' in recipe_dict['materials']:
                model.liquids = [liquid_dict for liquid_dict in recipe_dict['materials']['liquids']]
        for state_dict in recipe_dict['process']:
            model.states.append(state_dict['state_name'])
            model.transitions.extend([{'trigger': trigger, 'source': state_dict['state_name'],
                                'dest': state_dict['transitions'][trigger]} 
                                 for trigger in ['on_success','on_fail']])
            state_detail = StateDetails(station_type=state_dict['station']['type'],
                                station_id=int(state_dict['station']['id']),
                                station_op=state_dict['station']['operation'],
                                station_process=state_dict['station']['process'])
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
    def name(self) -> str:
        return self._model_proxy.name

    @property
    def id(self) -> int:
        return self._model_proxy.exp_id

    @property
    def solids (self) -> List[Dict]:
        return self._model_proxy.solids

    @property
    def liquids (self) -> List[Dict]:
        return self._model_proxy.liquids

    @property
    def model(self) -> RecipeModel:
        return self._model_proxy.model

    @property
    def current_state(self) -> str:
        return self._model_proxy.current_state

    def advance_state(self, success: bool) -> None:
        self._update_recipe_sm_state()
        self._station_sm.on_success() if success else self._station_sm.on_fail()
        self._model_proxy.current_state = self._station_sm.state
        self._logRecipe(f"Current state advanced to {self._station_sm.state}")

    def is_complete(self) -> bool:
        return self.current_state == 'end_state'
    
    def is_faulty(self) -> bool:
        return self.current_state == 'faulty_state'

    def get_current_process(self) -> StationOpDescriptor:
        if not self.is_complete():
            self._update_recipe_sm_state()
            state_details = self._model_proxy.state_map[self.current_state]
            return state_details.station_process

    def get_current_task_op(self) -> StationOpDescriptor:
        if not self.is_complete():
            self._update_recipe_sm_state()
            state_details = self._model_proxy.state_map[self.current_state]
            return StationFactory.create_op_from_dict(dict(state_details.station_op))

    def get_current_station(self) -> Tuple[str, int]:
        if not self.is_complete():
            self._update_recipe_sm_state()
            state_details = self._model_proxy.state_map[self.current_state]
            return state_details.station_type, state_details.station_id

    def get_next_station(self, success: bool) -> Tuple[str, int]:
        self._update_recipe_sm_state()
        self._station_sm.on_success() if success else self._station_sm.on_fail()        
        if self._station_sm.state != 'end_state':
            state_details = self._model_proxy.state_map[self._station_sm.state]
            return state_details.station_type, state_details.station_id
        else:
            return 'end', None

    def _update_recipe_sm_state(self):
        self._station_sm.state = self.current_state
        
    def _logRecipe(self, message: str):
        print(f'Recipe [{self.id}]: ' + message)


