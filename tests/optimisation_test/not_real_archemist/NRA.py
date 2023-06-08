from typing import Dict
import yaml
import math as m
from archemist.core.persistence.recipe_files_watchdog import RecipeFilesWatchdog
from archemist.core.util import Location
from archemist.core.state.recipe import Recipe
from archemist.core.state.batch import Batch
from mongoengine import connect

class NotRealArchemist():
    def __init__(self) -> None:
        with open('/home/satheesh/ARChemeist_ws/src/archemist/tests/optimisation_test/algae_bot_recipe.yaml') as fs:
            self._recipe_doc = yaml.load(fs, Loader=yaml.SafeLoader)
        bat = Batch.from_arguments(31,2,Location(2,7,'table_frame'))
        self.params = {}
        keys = ['dye_A', 'dye_B']
        for key in keys:
            self.params[key] = None
        recipe = Recipe.from_dict(self._recipe_doc)
        self._find_parameters(self._recipe_doc)
        #recipe.set_end_state()
        bat.attach_recipe(recipe)
        self.result = self._function()
        with open('/home/satheesh/ARChemeist_ws/src/archemist/tests/optimisation_test/result.yaml', 'w') as file:
            yaml.dump(self.result, file, default_flow_style=None)

    def _find_parameters(self, recipe_dict: Dict):
        if isinstance(recipe_dict, dict):
            for key, value in recipe_dict.items():
                if key == 'dye_A':
                    self.params[key] = value
                elif key == 'dye_B':
                    self.params[key] = value
                else:
                    self._find_parameters(value)
        elif isinstance(recipe_dict, list):
            for item in recipe_dict:
                self._find_parameters(item)

    def _function(self) -> Dict:
        result = {}
        x = self.params['dye_A']
        y = self.params['dye_B']
        z = m.sqrt(float(x)**2 + float(y)**2)
        result['output'] = z
        return result
        
if __name__ == "__main__":
    connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    NotRealArchemist()