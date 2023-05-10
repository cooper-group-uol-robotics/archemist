# thread to watch recipe queue
# thread to watch completed 
import time
#from archemist.core.optimisation.recipe_generator import RecipeGenerator
from archemist.core.persistence.recipe_files_watchdog import RecipeFilesWatchdog
from optimisation_manager import OptimisationManager
from pathlib import Path
from threading import Thread

class OptimizationHandler:
    def __init__(self, recipe_dir) -> None:
        # self._manager = optimization_manager
        # self._recipe_generator = recipe_generator
        self.recipe_path = recipe_dir
        self._watch_optimization_thread = Thread(target=self._watch_optimization_complete)
        self._watch_optimization_thread.start()
        self._watch_recipe_thread = Thread(target=self._watch_recipe_queue)
        self._watch_recipe_thread.start()

    def _watch_optimization_complete(self):
        pass

    def _watch_recipe_queue(self):
        self.recipe_watcher = RecipeFilesWatchdog(self.recipe_path)
        self.recipe_watcher.start()
        self._recipe_queue = self.recipe_watcher.recipes_queue
        print(self._recipe_queue)
        time.sleep(1)


if __name__ == '__main__':
    cwd_path = Path.cwd()
    print(cwd_path)
    recipes_dir = Path.joinpath(cwd_path, "examples/algae_bot_workflow/recipes")
    x=OptimizationHandler(recipes_dir)


