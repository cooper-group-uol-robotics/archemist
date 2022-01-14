from archemist.persistence.yParser import Parser
from transitions import Machine

class Recipe:
    def __init__(self, recipe_doc: dict):
        self._name = recipe_doc['name']
        self._id = recipe_doc['id']
        self._station_op_descriptors = [stationOp for station_dict in recipe_doc['stations'] for stationOp in station_dict['stationOps']]
        self._solids = recipe_doc['materials']['solids']
        self._liquids = recipe_doc['materials']['liquids']
        states = [state_dict['state_name'] for state_dict in recipe_doc['workflowSM']]
        transitions = [{'trigger': trigger, 'source': state_dict['state_name'], 'dest': state_dict[trigger]} 
                        for state_dict in recipe_doc['workflowSM'] for trigger in ['onSuccess','onFail']]
        self._station_flow = Machine(states=states,initial=recipe_doc['current_state'], transitions=transitions)

    @property
    def name(self):
        return self._name

    @property
    def id(self):
        return self._id

    @property
    def station_op_descriptors (self):
        return self._station_op_descriptors

    @property
    def solids (self):
        return self._solids

    @property
    def liquids (self):
        return self._liquids

    @property
    def station_flow (self):
        return self._station_flow

    def get_current_state(self):
        return self._station_flow.state

    def advance_state(self, success: bool):
        if success:
            self._station_flow.onSuccess()
        else:
            self._station_flow.onFail()
        self._logRecipe('Current state advanced to ' + self.station_flow.state)

    def is_complete(self):
        return self._station_flow.state == 'end'

    def get_current_task_op(self):
        if not self.is_complete():
            _, current_op_name = self._station_flow.state.split('.')
            current_op_dict = next(op_dict for op_dict in self._station_op_descriptors if op_dict['type'] == current_op_name)
            station_op_obj = Parser.str_to_class_station(current_op_name)
            station_op_output_obj = Parser.str_to_class_station(current_op_dict['output'])
            return station_op_obj(current_op_dict['properties'], station_op_output_obj())

    def _logRecipe(self, message: str):
        print(f'Recipe [{self._id}]: ' + message)


