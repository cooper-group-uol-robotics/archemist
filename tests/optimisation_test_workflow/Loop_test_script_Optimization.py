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
        self._find_max()
        self._values_dict = {}
        self._values_dict = self.generate_random_values()

        iter = 0
        self._dye_A = []
        self._dye_B = []
        self._result = []

        while True:
            self.params = {}
            keys = ['dye_A', 'dye_B', 'water']
            for key in keys:
                self.params[key] = None
            self._find_parameters(self._values_dict)
            self.update_function()
            if not iter == 0:
                within_threshold = self.fitness_function(self._values_dict)
                if within_threshold:
                    break

            result_data_pd = pd.DataFrame(self._values_dict)
            self._optimizer.update_model(result_data_pd)
            values_from_optimizer = self._optimizer.generate_batch()
            print(values_from_optimizer)
            self._values_dict = values_from_optimizer.to_dict(orient='list')
            iter += 1
        self.plot_list_elements()
    


    def plot_list_elements(self):
        dye_A = []
        dye_B = []
        result = []
        for sublist in range(len(self._dye_A)):
            dye_A.append(sum(self._dye_A[sublist])/len(self._dye_A[sublist]))
            dye_B.append(sum(self._dye_B[sublist])/len(self._dye_B[sublist]))
            result.append(sum(self._result[sublist])/len(self._result[sublist]))
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot the surface
        ax.plot_surface(self._A, self._B, self._Z, cmap='viridis', alpha=0.5)
        ax.plot(dye_A, dye_B, result, marker='o', linestyle='-', color='b', label='optimizer_data')

        for i in range(len(dye_A) - 1):
            dx = dye_A[i + 1] - dye_A[i]
            dy = dye_B[i + 1] - dye_B[i]
            dz = result[i + 1] - result[i] # z-coordinate where the line plot is located
            ax.quiver(dye_A[i], dye_B[i], result[i], dx, dy, dz, color='b', arrow_length_ratio=0.03)

        ax.set_xlabel('dye_A')
        ax.set_ylabel('dye_B')
        ax.set_zlabel('Z Label')
        ax.set_title(f'3D Function Plot - with max_value:{self._max_value}')
        plt.legend()
        plt.show()

    
    def fitness_function(self, current_values_dict):
        current_mean = sum(current_values_dict['red_intensity'])/len(current_values_dict['red_intensity'])
        print("current_mean", current_mean)
        if abs(self._max_value - current_mean) < 0.01:
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
            out = -1*((float((dye_A[index])**2))+ float((dye_B[index])**2))
            # out = np.sin(np.sqrt(dye_A[index]**2 + dye_B[index]**2)) * np.cos(dye_A[index] + dye_B[index])
            # out = 100*(np.sqrt(abs(dye_B[index] - 0.01*(dye_A[index]**2)))+(0.01*abs(dye_A[index]+10))) # Bukin Function N. 6
            # out = ((np.cos(np.sin(np.abs((dye_A[index])**2 - (dye_B[index])**2)))**2 - 0.5) / ((1 + 0.001 * ((dye_A[index])**2 + (dye_B[index])**2))**2) + 0.5 ) # schaffer function 
        

            self._values_dict['red_intensity'].append(out)

        self._dye_A.append(dye_A)
        self._dye_B.append(dye_B)
        self._result.append(self._values_dict['red_intensity'])

    def _find_max(self):
        self._A = np.linspace(0, 1, 100)
        self._B = np.linspace(0, 1, 100)
        self._A, self._B = np.meshgrid(self._A, self._B)
        self._Z = -1*((((self._A)**2))+ (self._B)**2)
        # self._Z =np.sin(np.sqrt(self._A**2 + self._B**2)) * np.cos(self._A + self._B)
        # self._Z = 100*(np.sqrt(abs((self._B) - 0.01*(self._A**2)))+(0.01*abs(self._A+10))) # Bukin Function N. 6
        # self._Z = ((np.cos(np.sin(np.abs((self._A)**2 - (self._B)**2)))**2 - 0.5) / ((1 + 0.001 * ((self._A)**2 + (self._B)**2))**2) + 0.5 )# schaffer function 
        
        self._max_value = np.max(self._Z)
        print("max", self._max_value)


if __name__ == "__main__":
    connect(db='archemist_opt_test', host='mongodb://localhost:27017', alias='archemist_state')
    Optimization_test()