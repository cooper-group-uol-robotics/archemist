import unittest
from pathlib import Path
from archemist.core.persistence.recipe_files_watchdog import RecipeFilesWatchdog
from time import sleep


class RecipeWatchdogTest(unittest.TestCase):

    def setUp(self):
        self._watch_path = Path.cwd()

    def tearDown(self):
        for file in self._watch_path.glob("recipe_*.yaml"):
            file.unlink()

    def test_watch_folder(self):
        recipe_watchdog = RecipeFilesWatchdog(self._watch_path)
        recipe_watchdog.start()

        # test empty queue
        self.assertFalse(recipe_watchdog.recipes_queue)

        # test recipes addition
        recipe_1_path = self._watch_path.joinpath("recipe_1.yaml")
        with open(recipe_1_path, "w") as recipe_file:
            recipe_file.write(" ")
            sleep(0.5)

        self.assertEqual(len(recipe_watchdog.recipes_queue), 1)
        self.assertEqual(recipe_watchdog.recipes_queue[0], recipe_1_path)

        recipe_2_path = self._watch_path.joinpath("recipe_2.yaml")
        with open(recipe_2_path, "w") as recipe_file:
            recipe_file.write(" ")
            sleep(0.5)

        self.assertEqual(len(recipe_watchdog.recipes_queue), 2)
        self.assertEqual(recipe_watchdog.recipes_queue[1], recipe_2_path)

        # test recipes removal
        recipe_2_path.unlink()
        sleep(0.5)

        self.assertEqual(len(recipe_watchdog.recipes_queue), 1)

        recipe_1_path.unlink()
        sleep(0.5)

        self.assertFalse(recipe_watchdog.recipes_queue)

    def test_watch_folder_with_existing_recipes(self):
        # create recipe files
        for i in range(1, 3):
            recipe_path = self._watch_path.joinpath(f"recipe_{i}.yaml")
            with open(recipe_path, "w") as recipe_file:
                recipe_file.write(" ")
                sleep(0.5)

        recipe_watchdog = RecipeFilesWatchdog(self._watch_path)
        recipe_watchdog.start()
        sleep(0.5)

        self.assertEqual(len(recipe_watchdog.recipes_queue), 2)
