from strictyaml import Map, Str, Int, Seq, Any, Optional, Float, Datetime, NullNone, MapPattern, dirty_load, Bool
from pkg_resources import resource_string
from typing import Dict
from pathlib import Path


class WorkflowSchemas:

    config_schema = Map(
        {
            'general': Map(
                {
                    'name': Str(),
                }
            ),
            'robots': NullNone() | Seq(Map(
                {
                    'type': Str(),
                    'id': Int(),
                    'handler': Str(),
                    Optional('location'): Map({'coordinates': Seq(Int()), 'descriptor': Str()}),
                    Optional('total_lot_capacity'): Int(),
                    Optional('onboard_capacity'): Int(),
                }
            )),
            'stations': Seq(Map(
                {
                    'type': Str(),
                    'id': Int(),
                    'location': Map({'coordinates': Seq(Int()), 'descriptor': Str()}),
                    'total_lot_capacity': Int(),
                    'handler': Str(),
                    'properties': NullNone() | MapPattern(Str(), Any()),
                    'materials': NullNone() | Map(
                        {
                            Optional('liquids'): Seq(Map(
                                {
                                    'name': Str(),
                                    'amount': Float(),
                                    'unit': Str(),
                                    'density': Float(),
                                    'density_unit': Str(),
                                    'details':  NullNone() | MapPattern(Str(), Any()),
                                    'expiry_date': Datetime()
                                }
                            )),
                            Optional('solids'): Seq(Map(
                                {
                                    'name': Str(),
                                    'amount': Float(),
                                    'unit': Str(),
                                    'expiry_date': Datetime(),
                                    'details':  NullNone() | MapPattern(Str(), Any())
                                }
                            ))
                        })
                }
            )),
            'workflow_input': Map(
                {
                    'location': Map({'coordinates': Seq(Int()), 'descriptor': Str()}),
                    'samples_per_batch': Int(),
                    'batches_per_lot': Int(),
                    'total_lot_capacity': Int(),
                    'lot_input_process': NullNone() | Map({
                        'type': Str(),
                        'args': NullNone() | MapPattern(Str(), Any())
                    })
                }),
            'workflow_output': Map(
                {
                    'location': Map({'coordinates': Seq(Int()), 'descriptor': Str()}),
                    'total_lot_capacity': Int(),
                    'lot_output_process': NullNone() | Map({
                        'type': Str(),
                        'args': NullNone() | MapPattern(Str(), Any())
                    }),
                    'lots_need_manual_removal': Bool()
                })
        })

    server_settings_schema = Map(
        {
            'db_name': Str(),
            'mongodb_host': Str(),
        }
    )

    recipe_schema = Map(
        {
            'general': Map(
                {
                    'name': Str(),
                    'id': Int(),
                }
            ),
            'steps': Seq(Map(
                {
                    'state_name': Str(),
                    'station': Map(
                        {
                            'type': Str(),
                            'id': Int(),
                            'process': Map(
                                {
                                    'type': Str(),
                                    'operations': NullNone() | Seq(Map(
                                        {
                                            'name': Str(),
                                            'op': Str(),
                                            'parameters': NullNone() | MapPattern(Str(), Any())
                                        }
                                    )),
                                    'args': NullNone() | MapPattern(Str(), Any())
                                }
                            )
                        }
                    ),
                    'transitions': Map(
                        {
                            'on_success': Str(),
                            'on_fail': Str()
                        }
                    )
                }
            ))
        }
    )


class YamlHandler:

    def _load_and_validate_schema(file_path: Path, schema: Map):
        with open(file_path, 'r') as config_file:
            yaml_str = config_file.read()
            return dirty_load(yaml_str, schema=schema, allow_flow_style=True)

    @staticmethod
    def load_config_file(file_path: Path) -> Dict:
        yaml_config = YamlHandler._load_and_validate_schema(
            file_path=file_path, schema=WorkflowSchemas.config_schema)
        return yaml_config.data

    @staticmethod
    def load_server_settings_file(file_path: Path) -> Dict:
        yaml_server_settings = YamlHandler._load_and_validate_schema(
            file_path=file_path, schema=WorkflowSchemas.server_settings_schema)
        return yaml_server_settings.data

    @staticmethod
    def load_recipe_file(file_path: Path) -> Dict:
        yaml_recipe = YamlHandler._load_and_validate_schema(
            file_path=file_path, schema=WorkflowSchemas.recipe_schema)
        return yaml_recipe.data

    @staticmethod
    def create_empty_config_file(file_path: Path):
        config_file_content = resource_string(
            'archemist.core.persistence.templates', 'workflow_config.yaml').decode('utf-8')
        config_file_content = config_file_content.replace('\r\n', '\r')
        file_path = file_path.joinpath('workflow_config.yaml')
        with open(file_path, 'w') as config_file:
            config_file.write(config_file_content)

    @staticmethod
    def create_sample_recipe_file(file_path: Path):
        recipe_file_content = resource_string(
            'archemist.core.persistence.templates', 'sample_recipe.yaml').decode('utf-8')
        recipe_file_content = recipe_file_content.replace('\r\n', '\r')
        file_path = file_path.joinpath('sample_recipe.yaml')
        with open(file_path, 'w') as recipe_file:
            recipe_file.write(recipe_file_content)

    @staticmethod
    def create_empty_server_settings_file(file_path: Path):
        recipe_file_content = resource_string(
            'archemist.core.persistence.templates', 'server_settings.yaml').decode('utf-8')
        recipe_file_content = recipe_file_content.replace('\r\n', '\r')
        file_path = file_path.joinpath('server_settings.yaml')
        with open(file_path, 'w') as recipe_file:
            recipe_file.write(recipe_file_content)
