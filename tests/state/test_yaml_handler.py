import unittest
from archemist.core.persistence.yaml_handler import YamlHandler
from pathlib import Path


class YamlHandlerTest(unittest.TestCase):
    def test_workflow_config(self):
        dir_path = Path(__file__).parent
        config_file_path = dir_path.joinpath("resources/testing_config_file.yaml")
        config_dict = YamlHandler.load_config_file(config_file_path)
        self.assertIsNotNone(config_dict)

    def test_recipe_file(self):
        dir_path = Path(__file__).parent
        config_file_path = dir_path.joinpath("resources/testing_recipe.yaml")
        config_dict = YamlHandler.load_recipe_file(config_file_path)
        self.assertIsNotNone(config_dict)


if __name__ == "__main__":
    unittest.main()
