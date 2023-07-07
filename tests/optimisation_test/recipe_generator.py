import pandas as pd
import yaml
from archemist.core.persistence.yaml_handler import YamlHandler
from nested_lookup import nested_update
from pathlib import Path
import logging
import re
from typing import Dict

class RecipeGenerator:
    def __init__(self, template_path, recipe_path) -> None:
        self._recipe_dict  = YamlHandler.load_recipe_file(template_path)
        self._new_recipe_path = recipe_path
        
        self._placeholder_keys_all = {}
        self._placeholder_keys_any = {}
        self._find_placeholders(self._recipe_dict)

    # Methods
    def generate_recipe(self, optimised_parameters_data, file_name):
        self._new_recipe_file_name = file_name
        _file_path = Path.joinpath(self._new_recipe_path, self._new_recipe_file_name)
        recipe_dict = self._recipe_dict
        optimised_parameters_dict = optimised_parameters_data.to_dict()
        if self._placeholder_keys_any:
            recipe_dict = self._update_recipe(recipe_dict,'any', optimised_parameters_dict)
        if self._placeholder_keys_all:
            recipe_dict = self._update_recipe(recipe_dict, 'all', optimised_parameters_dict)
        self._write_recipe(recipe_dict, _file_path)
    
    # internal functions
    def _find_placeholders(self, recipe_dict: Dict):
        if isinstance(recipe_dict, dict):
            for key, value in recipe_dict.items():
                if isinstance(value, str) and re.search(r"^{{*.*}}$", value):
                    strings = re.split("\.", value)
                    data_type = strings[0].lstrip("{")
                    out_type = strings[1].rstrip("}")
                    self._sort_keys(data_type, out_type, key)
                else:
                    self._find_placeholders(value)
        elif isinstance(recipe_dict, list):
            for item in recipe_dict:
                self._find_placeholders(item)

    def _sort_keys(self, data_type , out_type, _key):
        if out_type == "all":
            if not data_type in self._placeholder_keys_all:
                self._placeholder_keys_all[data_type] = []
            self._placeholder_keys_all[data_type].append(_key)
        elif out_type == "any":
            if not data_type in self._placeholder_keys_any:
                self._placeholder_keys_any[data_type] = []
            self._placeholder_keys_any[data_type].append(_key)
             
    def _update_recipe(self, recipe_dict: Dict, type, param_dict: Dict):
        if type == 'all':
            placeholder_dict = self._placeholder_keys_all
        elif type == 'any':
            placeholder_dict = self._placeholder_keys_any
        for data_type, keys in placeholder_dict.items():
            for key in keys:
                for p_key, p_values in param_dict.items():
                    if key == p_key:
                        if type == 'all':
                            value = list(p_values.values())
                            if data_type == "float":
                                value = [float(i) for i in value]
                            elif data_type == "int":
                                value = [int(i) for i in value]
                            elif data_type == "bool":
                                value = [bool(i) for i in value]
                            elif data_type == "str":
                                value = [str(i) for i in value]
                        elif type == 'any':
                            if data_type == "float":
                                value = float(p_values[0])
                            elif data_type == "int":
                                value = int(p_values[0])
                            elif data_type == "bool":
                                value = bool(p_values[0])
                            elif data_type == "str":
                                value = str(p_values[0])
                        else:
                            logging.error('improper placeholder type')
                        recipe_dict = nested_update(recipe_dict, key = key, value = value)
                    elif key != p_key:
                        pass
        return recipe_dict    

    def _write_recipe(self, recipe_dict: Dict, path):
        with open(path, 'w') as file:
            yaml.dump(recipe_dict, file, default_flow_style=None)

