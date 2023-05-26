from typing import Dict
import yaml
from archemist.core.persistence.recipe_files_watchdog import RecipeFilesWatchdog
from archemist.core.util import Location
from state import Batch, Recipe






class NotRealArchemist():
    def __init__(self) -> None:
        with open('/home/satheesh/ARChemeist_ws/src/archemist/tests/optimisation_test/algae_bot_recipe.yaml') as fs:
            recipe_doc = yaml.load(fs, Loader=yaml.SafeLoader)
        bat = Batch.from_arguments(31,2,Location(2,7,'table_frame'))
        recipe = Recipe.from_dict(recipe_doc)
        recipe.set_end_state()
        bat.attach_recipe(recipe)
        self.params = self._get_params()
        result = self._function()
        bat.add_result(result)



    def _get_params(self):
        pass

    
    def _function(self):
        pass


        


    