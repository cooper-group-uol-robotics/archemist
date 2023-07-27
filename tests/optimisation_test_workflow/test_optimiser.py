import unittest
from archemist.core.persistence.object_factory import OptimizationFactory
from archemist.core.optimisation.optimisation_records import OptimizationRecords
from mongoengine import connect
from pathlib import Path
import yaml
import pandas as pd
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class OptimiserTest(unittest.TestCase):
    def setUp(self) -> None:
        connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    
    def test_optimiser(self):
        workflow_dir = Path.joinpath(Path.cwd(), "tests/optimisation_test_workflow")
        config_file = Path.joinpath(workflow_dir, "config_files/optimization_config.yaml")
        config_dict = {}
        with open(config_file, 'r') as fs:
            config_dict = yaml.load(fs, Loader=yaml.SafeLoader)
        opt_state = OptimizationRecords.from_dict(config_dict)
        optimizer = OptimizationFactory.create_from_records(opt_state)

        # Plotting related init
        plotting_required = True       
        dye_A = np.linspace(0, 1, 100)
        dye_B = np.linspace(0, 1, 100)
        dye_A, dye_B = np.meshgrid(dye_A, dye_B)
        red_intensity = 1 - ((np.cos(np.sin(np.abs((dye_A)**2 - (dye_B)**2)))**2 - 0.5) / ((1 + 0.001 * ((dye_A)**2 + (dye_B)**2))**2) + 0.5) 
        max_red_intensity = np.max(red_intensity)
        line_plot_data = {}
        line_plot_data["dye_A"] = []
        line_plot_data["dye_B"] = []
        line_plot_data["red_intensity"] =  []
        
        def calc_intensity(dye_A: float, dye_B: float):
            # return -1*np.sqrt(dye_A**2 + dye_B**2)
            return 1 - ((np.cos(np.sin(np.abs((dye_A)**2 - (dye_B)**2)))**2 - 0.5) / ((1 + 0.001 * ((dye_A)**2 + (dye_B)**2))**2) + 0.5 )

        def mean(list: list):
            return (sum(list)/len(list))
        
        i = 0        
        while True:
            if i == 0:
                input_vals: pd.DataFrame = optimizer.generate_random_values()
                self.assertFalse(input_vals.empty)
                self.assertEqual(input_vals.shape, (6,2))
            else:
                input_vals: pd.DataFrame = optimizer.generate_batch()
                self.assertFalse(input_vals.empty)
                self.assertEqual(input_vals.shape, (6,2))
            input_vals["red_intensity"] = input_vals.apply(lambda row: calc_intensity(row["dye_A"], row["dye_B"]), axis=1)
            mean_red_intensity = mean(input_vals["red_intensity"].values.flatten().tolist())

            # for plotting
            line_plot_data["dye_A"].append(mean(input_vals["dye_A"].values.flatten().tolist()))
            line_plot_data["dye_B"].append(mean(input_vals["dye_B"].values.flatten().tolist()))
            line_plot_data["red_intensity"].append(mean_red_intensity)
            print(input_vals["red_intensity"])

            # fitness check
            print(mean_red_intensity)
            if abs(max_red_intensity - mean_red_intensity) < 0.01:
                break
            self.assertEqual(input_vals.shape, (6,3))
            optimizer.update_model(input_vals)
            i+=1
        self.assertLessEqual(abs(mean(input_vals["red_intensity"])- max_red_intensity), 0.01)
        
        if plotting_required:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.plot_surface(dye_A, dye_B, red_intensity, cmap='viridis', alpha=0.5)
            ax.plot(line_plot_data["dye_A"], line_plot_data["dye_B"], line_plot_data["red_intensity"], marker='o', linestyle='-', color='b', label='optimizer_data')
            ax.set_xlabel('dye_A')
            ax.set_ylabel('dye_B')
            ax.set_zlabel('Z Label')
            ax.set_title(f'3D Function Plot - with max_value:{max_red_intensity}')
            plt.legend()
            plt.show()


if __name__ == "__main__":
    unittest.main()
