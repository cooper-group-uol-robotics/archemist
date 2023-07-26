import unittest
from archemist.core.persistence.object_factory import OptimizationFactory
from archemist.core.state.optimisation_state import OptimizationState
from mongoengine import connect
from pathlib import Path
import yaml
import pandas as pd
import math

class OptimiserTest(unittest.TestCase):
    def setUp(self) -> None:
        connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    
    def test_optimiser(self):
        workflow_dir = Path(".")
        config_file = Path.joinpath(workflow_dir, "config_files/optimization_config.yaml")
        config_dict = {}
        with open(config_file, 'r') as fs:
            config_dict = yaml.load(fs, Loader=yaml.SafeLoader)
        opt_state = OptimizationState.from_dict(config_dict['optimizer'])

        optimizer = OptimizationFactory.create_from_dict(opt_state,config_dict)
        def calc_intensity(dye_A: float, dye_B: float):
            return round(-1*math.sqrt(dye_A**2 + dye_B**2))

        for i in range(100):
            if i == 0:
                input_vals: pd.DataFrame = optimizer.generate_random_values()
                self.assertFalse(input_vals.empty)
                self.assertEqual(input_vals.shape, (6,2))
            else:
                input_vals: pd.DataFrame = optimizer.generate_batch()
                self.assertFalse(input_vals.empty)
                self.assertEqual(input_vals.shape, (6,2))
            input_vals["red_intensity"] = input_vals.apply(lambda row: calc_intensity(row["dye_A"], row["dye_B"]), axis=1)
            print(input_vals)
            self.assertEqual(input_vals.shape, (6,3))
            optimizer.update_model(input_vals)
    
if __name__ == "__main__":
    unittest.main()
