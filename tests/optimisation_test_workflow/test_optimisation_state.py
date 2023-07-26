import unittest
from mongoengine import connect
from pathlib import Path
import yaml

from archemist.core.state.optimisation_state import OptimizationState

class OptimisationStateTest(unittest.TestCase):
    def setUp(self) -> None:
        connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')

    def test_optimisation_state(self):
        # construct optimisation_state
        workflow_dir = Path(".")
        config_file = Path.joinpath(workflow_dir, "config_files/optimization_config.yaml")
        config_dict = {}
        with open(config_file, 'r') as fs:
            config_dict = yaml.load(fs, Loader=yaml.SafeLoader)
        opt_state = OptimizationState.from_dict(config_dict['optimizer'])

        # test fields
        self.assertEqual(opt_state.optimizer_module, "bayesopt_optimiser")
        self.assertEqual(opt_state.optimizer_class, "BayesOptOptimizer")
        self.assertDictEqual(opt_state.objective_variable, {"SampleColorOpDescriptor":"red_intensity"})
        self.assertListEqual(opt_state.decision_variables, [{"CSCSVJobOpDescriptor": ["dispense_info"]}])
        self.assertDictEqual(opt_state.optimizer_hyperparameters, {"allow_duplicate_points": True})
        self.assertEqual(opt_state.recipe_template_name, "algae_bot_recipe_template")
        self.assertEqual(opt_state.generated_recipes_prefix, "algae_bot_recipe")
        self.assertEqual(opt_state.max_recipe_count, 3)
        self.assertEqual(len(opt_state.batches_seen), 0)

if __name__ == "__main__":
    unittest.main()

