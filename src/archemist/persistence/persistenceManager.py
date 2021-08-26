import persistence.dbHandler
import persistence.fsHandler
import persistence.yParser
import os


class persistenceManager:
    def __init__(self):
        self.dbhandler = persistence.dbHandler.dbHandler()
        self.fshandler = persistence.fsHandler.FSHandler()
        self.parser = persistence.yParser.Parser()
        self.loadedConfig = None

    def loadConfig(self):
        self.dbhandler.importConfig()
        print("Config imported at: " + self.dbhandler.getConfig()["timestamp"])
        config = dict()
        config = {'workflow': self.dbhandler.getConfig()}
        self.loadedConfig = config
        return config

    def overwriteConfig(self):
        __location__ = os.path.realpath(
            os.path.join(os.getcwd(), os.path.dirname(__file__)))
        config = dict()
        config = {'workflow': self.dbhandler.getConfig()}
        self.fshandler.overwriteYamlFile(config, os.path.join(
            __location__, 'workflowConfigs/config.yaml'))
        self.loadedConfig = config
        return config
