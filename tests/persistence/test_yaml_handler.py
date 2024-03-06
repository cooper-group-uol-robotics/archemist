import unittest
from archemist.core.persistence.yaml_handler import YamlHandler
from pathlib import Path
from strictyaml.exceptions import YAMLValidationError


class YamlHandlerTest(unittest.TestCase):

    def setUp(self):
        self._resource_path = Path.joinpath(Path.cwd(), "tests/persistence/resources")

    def test_config_file_loading(self):

        # test good config file
        good_config_path = self._resource_path.joinpath("good_wf_config.yaml")
        good_config = YamlHandler.load_config_file(good_config_path)
        self.assertIsNotNone(good_config)

        # test bad config file
        bad_config_path = self._resource_path.joinpath("bad_wf_config.yaml")
        with self.assertRaises(YAMLValidationError):
            YamlHandler.load_config_file(bad_config_path)

        # test good recipe file
        good_recipe_path = self._resource_path.joinpath("good_recipe.yaml")
        good_recipe = YamlHandler.load_recipe_file(good_recipe_path)
        self.assertIsNotNone(good_recipe)

        # test bad recipe file
        bad_recipe_path = self._resource_path.joinpath("bad_recipe.yaml")
        with self.assertRaises(YAMLValidationError):
            YamlHandler.load_config_file(bad_recipe_path)

        # test good server config file
        good_server_config_path = self._resource_path.joinpath("good_server_config.yaml")
        good_server_config = YamlHandler.load_server_settings_file(good_server_config_path)
        self.assertIsNotNone(good_server_config)

        # test bad recipe file
        bad_server_config_path = self._resource_path.joinpath("bad_server_config.yaml")
        with self.assertRaises(YAMLValidationError):
            YamlHandler.load_server_settings_file(bad_server_config_path)


if __name__ == '__main__':
    unittest.main()
