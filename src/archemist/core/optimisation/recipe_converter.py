import pandas as pd
import yaml
from archemist.core.persistence.yaml_handler import YamlHandler
from nested_lookup import nested_update
from pathlib import Path
import logging
import re

class RecipeUpdate:
    def __init__(self, template_path, recipe_path) -> None:
        self.recipe_dict  = YamlHandler.load_recipe_file(template_path)
        self.recipe_dir = recipe_path
        self.placeholders = []
        self.placeholder_keys_all = []
        self.placeholder_keys_any = []
        self.find_placeholders(self.recipe_dict)

    def find_placeholders(self, recipe_dict):
        if isinstance(recipe_dict, dict):
            for key, value in recipe_dict.items():
                if isinstance(value, str) and re.search(r"^{{*.*}}$", value):
                    _strings = re.split("\.", value)
                    _data_type = _strings[0].lstrip("{")
                    _out_type = _strings[1].rstrip("}")
                    if _out_type == "all":
                        self.placeholder_keys_all.append(key)
                        self.placeholders.append(value)
                    elif _out_type == "any":
                        self.placeholder_keys_any.append(key)
                        self.placeholders.append(value)
                else:
                    self.find_placeholders(value)
        elif isinstance(recipe_dict, list):
            for item in recipe_dict:
                self.find_placeholders(item)
    
    def generate_recipe(self, _pd):
        _optimised_parameters_dict = _pd.to_dict()
        if len(self.placeholder_keys_any) != 0:
            self.recipe_dict = self._update_recipe(self.recipe_dict,'any', _optimised_parameters_dict)
        else:
            print('{{*.any}} place holders NOT found')
            
        if len(self.placeholder_keys_all) != 0:
            self.recipe_dict = self._update_recipe(self.recipe_dict, 'all', _optimised_parameters_dict)
        else:
            print('{{*.all}} place holders NOT found')    
        
        self._write_recipe(self.recipe_dict, self.recipe_dir)
             
    def _update_recipe(self, recipe_dict, _type, param_dict):
        _keys_optimised_parameters_dict = list(param_dict.keys())
        if _type == 'all':
           _placeholder_keys = self.placeholder_keys_all
        elif _type == 'any':
            _placeholder_keys = self.placeholder_keys_any
        else:
            logging.error('improper placeholder type')
        for _key in _placeholder_keys:
            for col in range(len(param_dict)):
                if _key == _keys_optimised_parameters_dict[col]:
                    if _type == 'all':
                        _value = list(param_dict[_keys_optimised_parameters_dict[col]].values())
                    elif _type == 'any':
                        pass
                        _value = float(param_dict[_keys_optimised_parameters_dict[col]][0])
                    else:
                        logging.error('improper placeholder type')
                    print(_value)
                    _data_update = nested_update(recipe_dict, key = str(_key), value = _value)
                    recipe_dict = _data_update
                elif _key != _keys_optimised_parameters_dict[col]:
                    pass
        return recipe_dict    

    def _write_recipe(self, _recipe_dict, path):
        with open(path, 'w') as file:
            print(_recipe_dict)
            yaml.dump(_recipe_dict, file)

# if __name__ == "__main__":
#     cwd_path = Path.cwd()
#     path_to_template_recipe = Path.joinpath(cwd_path, "tests/optimisation_test/algae_bot_recipe_template.yaml")
#     path_to_recipes_dir = Path.joinpath(cwd_path, "tests/optimisation_test/algae_bot_recipe.yaml")
#     optimised_parameters = {'water': [29.0, 32.0, 34.0, 37.0, 40.0, 43.0], 'dye_A': [0.3, 0.33, 0.35, 0.38, 0.41, 0.44], 'dye_B': [0.31, 0.34, 0.36, 0.39, 0.42, 0.45]}
#     _pd = pd.DataFrame(optimised_parameters)

#     recipe_converter = RecipeUpdate(path_to_template_recipe, path_to_recipes_dir)
#     recipe_converter.generate_recipe(_pd)







