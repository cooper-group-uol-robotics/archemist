import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from collections import deque
from pathlib import Path


class RecipesDirHandler(FileSystemEventHandler):
    """Decription FIXME"""

    def __init__(self):
        self.recipes_queue = deque()

    def on_created(self, event):
        recipe_file = Path(event.src_path)
        if recipe_file.name[-5:] == ".yaml":
            print(f"recipe file was added: {recipe_file.name}")
            self.recipes_queue.append(recipe_file)

    def on_deleted(self, event):
        recipe_file = Path(event.src_path)
        if recipe_file in self.recipes_queue:
            print(f"recipe file was removed: {recipe_file.name}")
            self.recipes_queue.remove(recipe_file)


class RecipeFilesWatchdog(Observer):
    """Decription FIXME"""

    def __init__(self, recipes_dir):
        super().__init__()
        self._recipes_dir = recipes_dir
        self._recipes_dir_handler = RecipesDirHandler()

        # add already existing recipes to the queue FIXME loggin
        print("checking recipes directory for existing recipes")

        recipes = Path(recipes_dir).glob("*")
        for recipe in recipes:
            if recipe.name[-5:] == ".yaml":
                print(f"recipe file was added: {recipe.name}")
                self._recipes_dir_handler.recipes_queue.append(recipe)

        self.schedule(
            event_handler=self._recipes_dir_handler,
            path=self._recipes_dir,
            recursive=False,
        )

    @property
    def recipes_queue(self):
        return self._recipes_dir_handler.recipes_queue

    def start(self):
        print("Recipe files watchdog started")
        print(f"watching folder: {Path(self._recipes_dir).resolve()}")
        return super().start()


if __name__ == "__main__":
    recipes_dir = "data"
    recipes_watcher = RecipeFilesWatchdog(recipes_dir)
    recipes_watcher.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        recipes_watcher.stop()
    recipes_watcher.join()
