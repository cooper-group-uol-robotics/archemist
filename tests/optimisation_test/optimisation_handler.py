import pandas as pd
import yaml
from yaml.loader import SafeLoader
import glob
from nested_lookup import nested_update
import os

class RecipeEditor:
    def __init__(self, path) -> None:
        self.files = glob.glob(path)
        super().__init__()

    def update_values(self, *args):
        _output_pd = args[0]
        print(_output_pd)    #TODO remove this
        _shape = _output_pd.shape
        _total_columns = _shape[1]
        with open(self.files[0]) as file:
            self.data = yaml.load(file, Loader=SafeLoader)
            print(self.data)
            for col in range(_total_columns):
                _key = _output_pd.columns[col]
                _values = _output_pd[_key].tolist()
                print(_key,_values)
                _data_update = nested_update(self.data, key = str(_key), value = _values)
                self.data = _data_update

    def create_optimized_recipe(self, path):
        with open(path, 'w') as file:
            print(self.data)
            yaml.dump(self.data, file, default_flow_style=None)

if __name__ == "__main__": 
    cwd = os.getcwd()
    path = "/home/satheesh/ARChemeist_ws/src/archemist/tests/optimisation_test/algae_bot_recipe_0.yaml"
    path2 = "/home/satheesh/ARChemeist_ws/src/archemist/tests/optimisation_test/algae_bot_recipe_0_opt.yaml"
    optimised_values = {'water': [29.0, 32.0, 34.0, 37.0, 40.0, 43.0], 'dye_A': [0.3, 0.33, 0.35, 0.38, 0.41, 0.44], 'dye_B': [0.31, 0.34, 0.36, 0.39, 0.42, 0.45]}
    
    _pd = pd.DataFrame(optimised_values)
    new_file_name = 'algae_bot_recipe'
    recipe = RecipeEditor(path)
    recipe.update_values(_pd)
    recipe.create_optimized_recipe(path2)





