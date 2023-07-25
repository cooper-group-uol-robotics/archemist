from typing import Dict
import yaml
import math as m
from archemist.core.state.optimisation_state import OptimizationState
from archemist.core.persistence.object_factory import OptimizationFactory
from archemist.core.optimisation.bayesopt_optimiser import BayesOptOptimizer
from archemist.core.persistence.yaml_handler import YamlHandler
from mongoengine import connect
from pathlib import Path
import random
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class Optimization_test:
    def __init__(self) -> None:
        cwd_path = Path.cwd()
        workflow_dir = Path.joinpath(cwd_path, "tests/optimisation_test_workflow")
        self._config_file = Path.joinpath(workflow_dir, "config_files/optimization_config.yaml")
        self._config_dict = YamlHandler.loadYamlFile(self._config_file)
        self._batch_size = self._config_dict['optimizer']['batch_size']
        self._optimization_state = OptimizationState.from_dict(self._config_dict['optimizer'])
        self._optimizer = BayesOptOptimizer(self._optimization_state,  self._config_dict)
        self._fun_plot()
        self._values_dict = {}
        self._values_dict = self.generate_random_values()
        cummilative_out = []
        iter = 0

        while True:
            self.params = {}
            keys = ['dye_A', 'dye_B', 'water']
            for key in keys:
                self.params[key] = None
            self._find_parameters(self._values_dict)
            self.update_function()
            cummilative_out.append(self._values_dict['red_intensity'])
            if not iter == 0:
                within_threshold = self.fitness_function(self._values_dict)
                if within_threshold:
                    break

            result_data_pd = pd.DataFrame(self._values_dict)
            self._optimizer.update_model(result_data_pd)
            values_from_optimizer = self._optimizer.generate_batch()
            self._values_dict = values_from_optimizer.to_dict(orient='list')
            iter += 1
        merged_list = []

        for sublist in cummilative_out:
            merged_list += sublist
        self.plot_list_elements(merged_list)
           
    def plot_list_elements(self, data_list):
        x = range(1, len(data_list) + 1)  
        y = data_list   
        plt.figure()
        plt.plot(x, y, marker='o', linestyle='-')
        plt.xlabel('Index')
        plt.ylabel('out')
        plt.title(f'result plot - with max_value:{self._max_value}')
        plt.grid(False)
        plt.show()

    
    def fitness_function(self, current_values_dict):
        current_mean = sum(current_values_dict['red_intensity'])/len(current_values_dict['red_intensity'])
        print("current_mean", current_mean)
        if abs(self._max_value - current_mean) < 0.0001:
            return True
        else:
            return False
 
    def generate_random_values(self):
        random_values = {}
        for key in self._config_dict['experiment']['components']:
            random_values[key] = []
            for element in range(self._batch_size):
                _val = random.uniform(self._config_dict['experiment']['components'][key]['lower_bound'],
                               self._config_dict['experiment']['components'][key]['upper_bound'])
                _val = round(_val,2)
                random_values[key].append(_val)
        return random_values
    
    def _find_parameters(self, value_dict: dict):
        if isinstance(value_dict, dict):
            for key, value in value_dict.items():
                if key == 'dye_A':
                    self.params[key] = value
                elif key == 'dye_B':
                    self.params[key] = value
                elif key == 'water':
                    self.params[key] = value
                else:
                    self._find_parameters(value)
        elif isinstance(value_dict, list):
            for item in value_dict:
                self._find_parameters(item)

    def update_function(self) -> Dict:
        dye_A = self.params['dye_A']
        dye_B = self.params['dye_B']
        self._values_dict['red_intensity'] = []
        for index in range(len(dye_A)):
            out = -(2*float((dye_A[index])**2))+ float((dye_B[index])**2)
            # out = round(m.sqrt(2*float((dye_A[index])**2) + float((dye_B[index])**2) + float((water[index])**2)),2)

            self._values_dict['red_intensity'].append(out)

    def _fun_plot(self):
        dye_A = np.linspace(0, 1, 100)
        dye_B = np.linspace(0, 1, 100)
        dye_A, dye_B = np.meshgrid(dye_A, dye_B)
        Z = (2*((dye_A)**2))+ (dye_B)**2
        self._max_value = np.max(Z)
        print("max_value", self._max_value)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot the surface
        ax.plot_surface(dye_A, dye_B, Z, cmap='viridis')

        # Add labels and title
        ax.set_xlabel('dye_A')
        ax.set_ylabel('dye_B')
        ax.set_zlabel('Z Label')
        ax.set_title(f'3D Function Plot - with max_value:{self._max_value}')

if __name__ == "__main__":
    connect(db='archemist_opt_test', host='mongodb://localhost:27017', alias='archemist_state')
    Optimization_test()
