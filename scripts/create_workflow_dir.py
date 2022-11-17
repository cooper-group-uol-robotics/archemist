import argparse
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
    base_path = Path(args.path) if args.path is not None else Path.cwd()
    workflow_dir_path = base_path.joinpath(args.name)
    workflow_dir_path.mkdir(parents=True,exist_ok=False)
    config_dir_path = workflow_dir_path.joinpath('config_files')
    config_dir_path.mkdir(exist_ok=False)
    recipe_dir_path = workflow_dir_path.joinpath('recipes')
    recipe_dir_path.mkdir(exist_ok=False)
    template_dir_path = workflow_dir_path.joinpath('templates')
    template_dir_path.mkdir(exist_ok=False)

    # add templates
    YamlHandler.create_empty_config_file(template_dir_path)
    YamlHandler.create_sample_recipe_file(template_dir_path)
    YamlHandler.create_empty_server_settings_file(template_dir_path)
    
