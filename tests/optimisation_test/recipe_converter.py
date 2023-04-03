import pandas as pd
import yaml
from archemist.core.persistence.yaml_handler import YamlHandler
from nested_lookup import nested_update
from pathlib import Path
import logging
import re

class RecipeUpdate:
    def __init__(self) -> None:
        self.placeholder_keys = []
        self.placeholder_keys_all = []
        self.placeholder_keys_any = []
        self.placeholders =[]
        super().__init__()

    def find_placeholders(self, recipe_dict):
        if isinstance(recipe_dict, dict):
            for key, value in recipe_dict.items():
                if isinstance(value, str) and re.search(r"^{{.+?all}}$", value):
                    self.placeholder_keys_all.append(key)
                    self.placeholders.append(value)
                elif isinstance(value, str) and re.search(r"^{{.+?any}}$$", value):
                    self.placeholder_keys_any.append(key)
                    self.placeholders.append(value)
                else:
                    self.find_placeholders(value)
        elif isinstance(recipe_dict, list):
            for item in recipe_dict:
                self.find_placeholders(item)
        return self.placeholder_keys_any, self.placeholder_keys_all
             
    def update_values(self, recipe_dict, _type, param_dict):
        _keys_optimised_parameters_dict = list(param_dict.keys())
        if _type == 'all':
           self.placeholder_keys = self.placeholder_keys_all
        elif _type == 'any':
            self.placeholder_keys = self.placeholder_keys_any
        else:
            logging.error('improper placeholder type')
        for _key in self.placeholder_keys:
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

    def generate_recipe(self, _recipe_dict, path):
        with open(path, 'w') as file:
            print(_recipe_dict)
            yaml.dump(_recipe_dict, file)

if __name__ == "__main__": 
    cwd_path = Path.cwd()
    recipe_path = Path.joinpath(cwd_path, "tests/optimisation_test/algae_bot_recipe_template.yaml")
    optimised_parameters = {'water': [29.0, 32.0, 34.0, 37.0, 40.0, 43.0], 'dye_A': [0.3, 0.33, 0.35, 0.38, 0.41, 0.44], 'dye_B': [0.31, 0.34, 0.36, 0.39, 0.42, 0.45]}
    _pd = pd.DataFrame(optimised_parameters)
    optimised_parameters_dict = _pd.to_dict()
    recipe_dict = YamlHandler.load_recipe_file(recipe_path)
    print(recipe_dict)
    recipe = RecipeUpdate()
    _keys_values, _keys_lists = recipe.find_placeholders(recipe_dict)
    if len(_keys_values) != 0:
        recipe_dict = recipe.update_values(recipe_dict,'any', optimised_parameters_dict)
    else:
        print('{{float.any}} place holders NOT found')

    if len(_keys_lists) != 0:
        recipe_dict = recipe.update_values(recipe_dict, 'all', optimised_parameters_dict)
    else:
        print('{{float.all}} place holders NOT found')    
    recipe.generate_recipe(recipe_dict, recipe_path)





