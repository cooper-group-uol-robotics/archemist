import argparse
import os
from pathlib import Path
from archemist.core.persistence.yaml_handler import YamlHandler

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='creates a workflow directory \
            to store config and recipe files')
    parser.add_argument('--name', dest='name', action='store', type=str,
                    required=True, help='name of the workflow')
    parser.add_argument('--path', dest='path', action='store', type=str,
                     help='path to the workflow directory')
    args = parser.parse_args()
    
    # create all the workflow directory 
    if not os.path.exists(args.path): os.makedirs(args.path)
    base_path = Path(args.path) if args.path is not None else Path.cwd()
    workflow_dir_path = base_path.joinpath(args.name)
    config_dir_path = workflow_dir_path.joinpath('config_files')
    recipe_dir_path = workflow_dir_path.joinpath('recipes')
    workflow_dir_path.mkdir(parents=False,exist_ok=False)
    config_dir_path.mkdir(parents=False,exist_ok=False)
    recipe_dir_path.mkdir(parents=False,exist_ok=False)

    # add workflow_config template
    config_file_path = config_dir_path.joinpath('workflow_config.yaml')
    YamlHandler.create_empty_config_file(config_file_path)

    # add recipe sample
    recipe_sample_path = workflow_dir_path.joinpath('recipe_sample.yaml')
    YamlHandler.create_sample_recipe_file(recipe_sample_path)
    
