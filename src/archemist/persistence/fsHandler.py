import yaml


class FSHandler:
    def loadYamlFile(self, filePath):
        fileStream = open(filePath, 'r')
        return yaml.load(fileStream, Loader=yaml.SafeLoader)

    def overwriteYamlFile(self, newDict, filePath):
        fileStream = open(filePath, 'w')
        yaml.dump(newDict, fileStream)
