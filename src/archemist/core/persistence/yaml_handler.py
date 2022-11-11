import yaml
from strictyaml import Map, Str, Int, Seq, Any, load, MapPattern

config_schema = Map(
    {
        'general': Map(
            {
                'name': Str(),
                'samples_per_batch': Int(),
                'default_batch_input_location': Map({'node_id': Int(), 'graph_id': Int()})
            }
        ),
        'robots': Seq(Map(
            {
                'type': Str(),
                'id': Int(),
                'batch_capacity': Int(),
                'handler': Str()
            }
        )),
        'stations': Seq(Map(
            {
                'type': Str(),
                'id': Int(),
                'location': Map({'node_id': Int(), 'graph_id': Int()}),
                'batch_capacity': Int(),
                'handler': Str(),
                'process_state_machine': Map({'type': Str(), 'args': Any()}),
                'parameters': Any()
            }
        ))
    })


class YamlHandler:

    @staticmethod
    def loadYamlFile(filePath):
        with open(filePath, 'r') as fs:
            return yaml.load(fs, Loader=yaml.SafeLoader)

    @staticmethod
    def load_config_file(file_path):
        with open(file_path, 'r') as config_file:
            yaml_str = config_file.read()
            return load(yaml_str, schema=config_schema)
