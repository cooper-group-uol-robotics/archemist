from typing import Dict
import yaml
import math as m
from archemist.core.persistence.recipe_files_watchdog import RecipeFilesWatchdog
from archemist.core.util import Location
from state import Batch, Recipe



class NotRealArchemist():
    def __init__(self) -> None:
        with open('/home/satheesh/ARChemeist_ws/src/archemist/tests/optimisation_test/algae_bot_recipe.yaml') as fs:
            self._recipe_doc = yaml.load(fs, Loader=yaml.SafeLoader)
        bat = Batch.from_arguments(31,2,Location(2,7,'table_frame'))
        recipe = Recipe.from_dict(self._recipe_doc)
        recipe.set_end_state()
        bat.attach_recipe(recipe)
        keys = ['dye_A', 'dye_B']
        self.params = {}
        for key in keys:
            self.params[key] = None
        result = self._function()
        bat.add_result(result)


    def _find_parameters(self, recipe_dict: Dict):
        params = {}
        if isinstance(recipe_dict, dict):
            for key, value in recipe_dict.items():
                if key == 'dye_A':
                    strings = re.split("\.", value)
                    data_type = strings[0].lstrip("{")
                    out_type = strings[1].rstrip("}")
                    self._sort_keys(data_type, out_type, key)
                else:
                    self._find_placeholders(value)
        elif isinstance(recipe_dict, list):
            for item in recipe_dict:
                self._find_placeholders(item)

    def _function(self):
        result = {}
        x = self.params['x']
        y = self.params['y']
        z = m.sqrt(x**2 + y**2)
        result['output'] = z
        return result


        


    