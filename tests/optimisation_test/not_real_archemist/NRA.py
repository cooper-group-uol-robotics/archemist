from typing import Dict
import yaml
import math as m
from archemist.core.persistence.recipe_files_watchdog import RecipeFilesWatchdog
from archemist.core.util import Location
from archemist.core.state.recipe import Recipe
from archemist.core.state.batch import Batch
from mongoengine import connect
import unittest

class NotRealArchemist():
    def __init__(self) -> None:
        with open('/home/uol/ARChemist_ws/src/archemist/tests/optimisation_test/recipes/algae_bot_recipe_1.yaml') as fs:
            self._recipe_doc = yaml.load(fs, Loader=yaml.SafeLoader)
        bat = Batch.from_arguments(31,2,Location(2,7,'table_frame'))
        self.params = {}
        keys = ['dye_A', 'dye_B', 'water']
        for key in keys:
            self.params[key] = None
        self._find_parameters(self._recipe_doc)

        self._result = self._function()
        self._result['batch_id'] = bat.id
        self._recipe_doc['result'] = self._result
        recipe = Recipe.from_dict(self._recipe_doc)
        #set end state for recipe
        recipe.advance_state(True)
        bat.attach_recipe(recipe)
        print(self._recipe_doc)
        with open('/home/uol/ARChemist_ws/src/archemist/tests/optimisation_test/algae_bot_recipe_result.yaml', 'w') as file:
            yaml.dump(self._recipe_doc, file, default_flow_style=None)
    

    def _find_parameters(self, recipe_dict: Dict):
        if isinstance(recipe_dict, dict):
            for key, value in recipe_dict.items():
                if key == 'dye_A':
                    self.params[key] = value
                elif key == 'dye_B':
                    self.params[key] = value
                elif key == 'water':
                    self.params[key] = value
                else:
                    self._find_parameters(value)
        elif isinstance(recipe_dict, list):
            for item in recipe_dict:
                self._find_parameters(item)

    def _function(self) -> Dict:
        result = {}
        dye_A = self.params['dye_A']
        dye_B = self.params['dye_B']
        water = self.params['water']
        result['output'] = []
        for index in range(len(dye_A)):
            out = m.sqrt(float(dye_A[index])**2 + float(dye_B[index])**2 + float(water[index])**2)
            result['output'].append(out)
        return result
        
if __name__ == "__main__":
    connect(db='archemist_opt_test', host='mongodb://localhost:27017', alias='archemist_state')
    NotRealArchemist()