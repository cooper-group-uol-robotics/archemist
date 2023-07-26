import unittest
from archemist.core.persistence.object_factory import OptimizationFactory
from archemist.core.optimisation.optimisation_records import OptimizationRecords
from mongoengine import connect
from pathlib import Path
import yaml
import pandas as pd
import math
import numpy as np


class OptimiserTest(unittest.TestCase):
    def setUp(self) -> None:
        connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    
    def test_optimiser(self):
        workflow_dir = Path.joinpath(Path.cwd(), "tests/optimisation_test_workflow")
        config_file = Path.joinpath(workflow_dir, "config_files/optimization_config.yaml")
        config_dict = {}
        max_red_intensity = 0.0
        plotting_required = True
        with open(config_file, 'r') as fs:
            config_dict = yaml.load(fs, Loader=yaml.SafeLoader)
        opt_state = OptimizationRecords.from_dict(config_dict)

        optimizer = OptimizationFactory.create_from_records(opt_state)
        def calc_intensity(dye_A: float, dye_B: float):
            return round(-1*math.sqrt(dye_A**2 + dye_B**2))

        def mean(list: list):
            return sum(list)/len(list)
        
        i = 0
        line_plot_data = {}
        line_plot_data["dye_A"] = line_plot_data["dye_B"] = line_plot_data["red_intensity"] =  []
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
            
            # for plotting

            line_plot_data["dye_A"].append(mean(input_vals["dye_A"].values.flatten().tolist()))
            line_plot_data["dye_B"].append(mean(input_vals["dye_B"].values.flatten().tolist()))
            line_plot_data["red_intensity"].append(mean(input_vals["red_intensity"].values.flatten().tolist()))
            
            # fitness check
            mean_red_intensity = sum(input_vals['red_intensity'])/len(input_vals['red_intensity'])
            print(i)
            if abs(max_red_intensity - mean_red_intensity) < 0.01:
                break
            self.assertEqual(input_vals.shape, (6,3))
            optimizer.update_model(input_vals)
            i+=1
        
        for value in input_vals["red_intensity"]:
            self.assertLessEqual((input_vals["red_intensity"][value] - max_red_intensity), 0.01)
        
        if plotting_required:
            dye_A = np.linspace(0, 1, 100)
            dye_B = np.linspace(0, 1, 100)
            dye_A, dye_B = np.meshgrid(dye_A, dye_B)
            red_intensity = -1*np.sqrt(dye_A**2 + dye_B**2)
            max_red_intensity = np.max(red_intensity)



if __name__ == "__main__":
    unittest.main()
