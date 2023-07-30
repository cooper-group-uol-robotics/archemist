import unittest
from mongoengine import connect
from pathlib import Path
from archemist.core.optimisation.optimisation_records import OptimisationRecords
from archemist.core.persistence.yaml_handler import YamlHandler
from archemist.core.optimisation.recipe_generator import RecipeGenerator
from archemist.core.persistence.persistence_manager import PersistenceManager
import pandas as pd


class RecipeGeneratorOpTest(unittest.TestCase):

    def test_recipe_generator(self):
        workflow_dir = Path("test_optimisation_workflow/")
        server_config_file_path = workflow_dir.joinpath(f'config_files/server_settings.yaml')
        _server_config = YamlHandler.load_server_settings_file(server_config_file_path)
        pm = PersistenceManager(_server_config['mongodb_host'], _server_config['db_name'])
        
        workflow_config_file = workflow_dir.joinpath("config_files/workflow_config.yaml")
        _state = pm.construct_state_from_config_file(workflow_config_file)
       
        optimisation_config_file = workflow_dir.joinpath("config_files/optimization_config.yaml")
        optimisation_config_dict = YamlHandler.loadYamlFile(optimisation_config_file)
        opt_records = OptimisationRecords.from_dict(optimisation_config_dict)
        
        recipes_dir = workflow_dir.joinpath("recipes")
        
        # delete any recipe files in the directory
        files = recipes_dir.glob("*.yaml")
        if files:
            for file in files:
                file.unlink()
        template_name = opt_records.recipe_template_name
        template_dir = recipes_dir.joinpath(f"template/{template_name}.yaml")
        template_recipe = YamlHandler.load_recipe_file(template_dir)
        recipe_generator = RecipeGenerator(
            template_dir,
            recipes_dir,
            opt_records.generated_recipes_prefix,
            _state,
        )
        self.assertEqual(recipe_generator._template_recipe_dict, template_recipe)
        self.assertEqual(recipe_generator._new_recipe_path, recipes_dir)
        self.assertEqual(recipe_generator._gen_recipe_name_prefix, "algae_bot_recipe")
        recipe_generator._find_placeholders(recipe_generator._template_recipe_dict)
        self.assertEqual(len(recipe_generator._placeholder_keys_all["float"]), 2)
        # self.assertEqual(len(recipe_generator._placeholder_keys_any["float"]), 1)
        recipe_id = -1  # may vary based on recipes in database
        self.assertEqual(recipe_generator._current_recipe_id, recipe_id)
        dispense_info = {
            "dye_A": [0.12, 0.15, 0.18, 0.21, 0.24, 0.27],
            "dye_B": [0.13, 0.16, 0.19, 0.22, 0.25, 0.28],
        }

        Benchmark_recipe = {
            "general": {"name": "algae_bot_recipe", "id": recipe_id + 1},
            "process": [
                {
                    "state_name": "pickup",
                    "station": {
                        "type": "InputStation",
                        "id": 11,
                        "operation": {"type": "InputStationPickupOp"},
                    },
                    "transitions": {
                        "on_success": "dispense_liquid",
                        "on_fail": "end_state",
                    },
                },
                {
                    "state_name": "dispense_liquid",
                    "station": {
                        "type": "ChemSpeedFlexStation",
                        "id": 12,
                        "operation": {
                            "type": "CSCSVJobOpDescriptor",
                            "properties": {
                                "dispense_info": {
                                    "dye_A": [0.12, 0.15, 0.18, 0.21, 0.24, 0.27],
                                    "dye_B": [0.13, 0.16, 0.19, 0.22, 0.25, 0.28],
                                }
                            },
                        },
                    },
                    "transitions": {"on_success": "analyse", "on_fail": "end_state"},
                },
                {
                    "state_name": "analyse",
                    "station": {
                        "type": "LightBoxStation",
                        "id": 13,
                        "operation": {"type": "SampleColorRGBOpDescriptor"},
                    },
                    "transitions": {"on_success": "dispose", "on_fail": "end_state"},
                },
                {
                    "state_name": "dispose",
                    "station": {
                        "type": "OutputStation",
                        "id": 14,
                        "operation": {"type": "OutputStationPlaceOp"},
                    },
                    "transitions": {"on_success": "end_state", "on_fail": "end_state"},
                },
            ],
        }

        generated_recipe = recipe_generator.generate_recipe(
            pd.DataFrame.from_dict(dispense_info)
        )
        
        self.assertEqual(Benchmark_recipe, generated_recipe)

        # final_recipe = template_recipe = YamlHandler.load_recipe_file(file_path)
        # self.assertEqual(Benchmark_recipe, final_recipe)

        # delete generated recipe files in the directory
        files = recipes_dir.glob("*.yaml")
        if files:
            for file in files:
                file.unlink()

if __name__ == "__main__":
    unittest.main()