import pandas as pd
import yaml
from archemist.core.persistence.yaml_handler import YamlHandler
from nested_lookup import nested_update
from pathlib import Path

class RecipeUpdate:
    def __init__(self) -> None:
        self.placeholder_keys = []
        self.placeholders =[]
        super().__init__()

    def find_placeholders(self, data):
        self._data = data
        if isinstance(self._data, dict):
            for key, value in self._data.items():
                if isinstance(value, str) and '{{' in value and '}}' in value:
                    self.placeholder_keys.append(key)
                    self.placeholders.append(value)
                else:
                    self.find_placeholders(value)
        elif isinstance(self._data, list):
            for item in self._data:
                self.find_placeholders(item)
             
    def update_values(self,data, *args):
        self.data = data
        _output_pd = args[0]
        _shape = _output_pd.shape
        _total_columns = _shape[1]
        for _key in self.placeholder_keys:
            for col in range(_total_columns):
                _head = _output_pd.columns[col]
                if _key == _head:
                    _values = _output_pd[_head].tolist()
                    print(_key,_values)
                    _data_update = nested_update(self.data, key = str(_key), value = _values)
                    self.data = _data_update
                elif _key != _head:
                    pass      

    def create_optimized_recipe(self, path):
        with open(path, 'w') as file:
            print(self.data)
            yaml.dump(self.data, file)

if __name__ == "__main__": 
    path = Path.cwd() / "tests" / "optimisation_test" / "algae_bot_recipe_template.yaml"
    optimised_values = {'water': [29.0, 32.0, 34.0, 37.0, 40.0, 43.0], 'dye_A': [0.3, 0.33, 0.35, 0.38, 0.41, 0.44], 'dye_B': [0.31, 0.34, 0.36, 0.39, 0.42, 0.45]}
    _pd = pd.DataFrame(optimised_values)
    new_file_name = 'algae_bot_recipe'
    data = YamlHandler.load_recipe_file(path)
    recipe = RecipeUpdate()
    recipe.find_placeholders(data)
    recipe.update_values(data, _pd)
    recipe.create_optimized_recipe(path)
    optimised_parameters = _pd.to_dict()




