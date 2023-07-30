import unittest
from mongoengine import connect
from pathlib import Path
import pandas as pd
import yaml

from archemist.core.optimisation.optimisation_records import OptimisationRecrods

class OptimisationRecordsTest(unittest.TestCase):
    def setUp(self) -> None:
        connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')

    def test_optimisation_state(self):
        # construct optimisation_state
        workflow_dir = Path.cwd()
        config_file = Path.joinpath(workflow_dir, "test_optimisation_workflow/config_files/optimization_config.yaml")
        config_dict = {}
        with open(config_file, 'r') as fs:
            config_dict = yaml.load(fs, Loader=yaml.SafeLoader)
        opt_records = OptimisationRecrods.from_dict(config_dict)

        # test fields
        self.assertEqual(opt_records.optimiser_module, "archemist.core.optimisation.bayesopt_optimiser")
        self.assertEqual(opt_records.optimiser_class, "BayesOptOptimizer")
        self.assertDictEqual(opt_records.optimiser_args["components"], config_dict["optimiser"]["args"]["components"])
        self.assertDictEqual(opt_records.optimiser_args["hyperparameters"], {"allow_duplicate_points": True})
        self.assertEqual(opt_records.optimiser_args["batch_size"], 6)
        self.assertDictEqual(opt_records.objective_variable, {"SampleColorOpDescriptor":["red_intensity"]})
        self.assertDictEqual(opt_records.decision_variables, {"CSCSVJobOpDescriptor": ["dispense_info"]})
        self.assertEqual(opt_records.recipe_template_name, "algae_bot_recipe_template")
        self.assertEqual(opt_records.generated_recipes_prefix, "algae_bot_recipe")
        self.assertEqual(opt_records.max_recipe_count, 3)
        self.assertEqual(len(opt_records.batches_seen), 0)

if __name__ == "__main__":
    unittest.main()

