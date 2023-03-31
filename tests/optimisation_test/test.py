import pandas as pd
import yaml
from archemist.core.persistence.yaml_handler import YamlHandler
from nested_lookup import nested_update
from pathlib import Path
import logging

optimised_parameters = {'water': [29.0, 32.0, 34.0, 37.0, 40.0, 43.0], 'dye_A': [0.3, 0.33, 0.35, 0.38, 0.41, 0.44], 'dye_B': [0.31, 0.34, 0.36, 0.39, 0.42, 0.45]}
_pd = pd.DataFrame(optimised_parameters)
optimised_parameters_dict = _pd.to_dict()


print(len(optimised_parameters))
print(len(optimised_parameters_dict))
_keys_optimised_parameters_dict = list(optimised_parameters_dict.keys())

print(_keys_optimised_parameters_dict[0])

print(optimised_parameters_dict[str(_keys_optimised_parameters_dict[0])])










