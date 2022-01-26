from archemist.persistence.dbObjProxy import DbObjProxy
from transitions import Machine

class Recipe:
    def __init__(self, recipe_doc: dict, db_doc_proxy: DbObjProxy):
        self._name = recipe_doc['name']
        self._id = recipe_doc['id']
        self._station_op_descriptors = [stationOp for station_dict in recipe_doc['stations'] for stationOp in station_dict['stationOps']]
        self._solids = recipe_doc['materials']['solids']
        self._liquids = recipe_doc['materials']['liquids']
        states = [state_dict['state_name'] for state_dict in recipe_doc['workflowSM']]
        transitions = [{'trigger': trigger, 'source': state_dict['state_name'], 'dest': state_dict[trigger]} 
                        for state_dict in recipe_doc['workflowSM'] for trigger in ['onSuccess','onFail']]
        self._station_flow = Machine(states=states,initial=recipe_doc['current_state'], transitions=transitions)
        
        self._db_proxy = db_doc_proxy

    @property
    def name(self):
        return self._name

    @property
    def id(self):
        return self._id

    @property
    def solids (self):
        return self._solids

    @property
    def liquids (self):
        return self._liquids

    @property
    def current_state(self):
        return self._db_proxy.get_nested_field('recipe.current_state')

    def advance_state(self, success: bool):
        self._station_flow.state = self.current_state
        if success:
            self._station_flow.onSuccess()
        else:
            self._station_flow.onFail()
        self._db_proxy.update_field('recipe.current_state', self._station_flow.state)
        self._logRecipe('Current state advanced to ' + self._station_flow.state)

    def is_complete(self):
        return self.current_state == 'end'

    def get_current_task_op_dict(self):
        if not self.is_complete():
            self._station_flow.state = self.current_state
            _, current_op_name = self._station_flow.state.split('.')
            current_op_dict = next(op_dict for op_dict in self._station_op_descriptors if op_dict['type'] == current_op_name)
            return current_op_dict
        
    def _logRecipe(self, message: str):
        print(f'Recipe [{self._id}]: ' + message)


