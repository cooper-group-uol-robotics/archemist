from typing import Dict
import yaml
import math as m
from object_factory import OptimizationFactory
from bayesopt_optimiser import BayesOptOptimizer
from archemist.core.persistence.yaml_handler import YamlHandler
from mongoengine import connect
from pathlib import Path
import random
import pandas as pd

class Optimization_test:
    def __init__(self) -> None:
        cwd_path = Path.cwd()
        workflow_dir = Path.joinpath(cwd_path, "tests/optimisation_test")
        self._config_file = Path.joinpath(workflow_dir, "config_files/optimization_config.yaml")
        self._config_dict = YamlHandler.loadYamlFile(self._config_file)
        self._batch_size = self._config_dict['optimizer']['batch_size']
        self._optimization_state = OptimizationFactory.create_from_dict(self._config_dict['optimizer'])
        self._optimizer = BayesOptOptimizer(self._optimization_state,  self._config_dict)
        values_from_optimizer = self._optimizer.generate_batch()
        print(values_from_optimizer)

        for iter in range(100):
            self._random_values_dict = {}
            self._random_values_dict = self.generate_random_values()

            self.params = {}
            keys = ['dye_A', 'dye_B', 'water']
            for key in keys:
                self.params[key] = None
            self._find_parameters(self._random_values_dict)
            self.update_function()
            print(f'iteration_{iter+1}: \n',self._random_values_dict)
            result_data_pd = pd.DataFrame(self._random_values_dict)
            self._optimizer.update_model(result_data_pd)
        
        for i in range(10):
            values_from_optimizer = self._optimizer.generate_batch()
            print(f'batch_{i+1}: \n', values_from_optimizer)



    def generate_random_values(self):
        # generate random pd frame from config file
        random_values = {}
        for key in self._config_dict['experiment']['components']:
            random_values[key] = []
            for element in range(self._batch_size):
                _val = random.uniform(self._config_dict['experiment']['components'][key]['lower_bound'],
                               self._config_dict['experiment']['components'][key]['upper_bound'])
                _val = round(_val,2)
                random_values[key].append(_val)
        return random_values

    
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

    def update_function(self) -> Dict:
        dye_A = self.params['dye_A']
        dye_B = self.params['dye_B']
        water = self.params['water']
        self._random_values_dict['red_intensity'] = []
        for index in range(len(dye_A)):
            out = round(m.sqrt(float(dye_A[index])**2 + float(dye_B[index])**2 + float(water[index])**2),2)
            self._random_values_dict['red_intensity'].append(out)
        
if __name__ == "__main__":
    connect(db='archemist_opt_test', host='mongodb://localhost:27017', alias='archemist_state')
    Optimization_test()