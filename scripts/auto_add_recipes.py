from archemist.core.persistence.persistence_manager import PersistenceManager
from archemist.core.persistence.objects_getter import RecipesGetter
from time import sleep
import shutil
from pathlib import Path

# connect to database and construct persistence manager
path_to_config = Path('C:/Users/ebrass/Documents/Code/ros_ws/src/archemist/examples/apc_workflow')
path_to_server_config = path_to_config.joinpath('config_files/server_settings.yaml')
pm = PersistenceManager(path_to_server_config)
pm.construct_workflow_from_db()

source_dir = Path('C:/Users/ebrass/Documents/Code/ros_ws/src/archemist/scripts/recipesss')
destination_dir = path_to_config.joinpath('recipes')
file_counter = 1

while True:
    recipes = RecipesGetter.get_recipes()
    if len(recipes) == 0 or all([recipe.is_complete() for recipe in recipes]):
        recipe_path = source_dir.joinpath(f"apc_recipe_paracetamol_{file_counter}.yaml")
        file_counter += 1
        # copy one recipe file to recipes folder
        shutil.copy(recipe_path, destination_dir)
        print(f'copied -> {recipe_path.name}')
    else:
        pass
    sleep(5)
