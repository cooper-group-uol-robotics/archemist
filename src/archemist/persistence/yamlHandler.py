import yaml


class YamlHandler:

    @staticmethod
    def loadYamlFile(filePath):
        with open(filePath, 'r') as fs:
            return yaml.load(fs, Loader=yaml.SafeLoader)

