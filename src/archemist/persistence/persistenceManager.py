import persistence.dbHandler
import persistence.fsHandler
import persistence.yParser
import os
from datetime import datetime


class persistenceManager:
    def __init__(self):
        self.dbhandler = persistence.dbHandler.dbHandler()
        self.fshandler = persistence.fsHandler.FSHandler()
        self.parser = persistence.yParser.Parser()
        self.loadedConfig = None
        self.client = self.dbhandler.getDBAccess()

    def changeStationProperty(self, station: str, newProperty, newValue):
        db = self.client.config
        conf = db.workflowConfig.find_one({"workflow": {"$exists": True}})
        if (station in conf["workflow"]["Stations"]):
            if (newProperty in conf["workflow"]["Stations"][station]):
                oldValue = conf["workflow"]["Stations"][station][newProperty]
                conf["workflow"]["Stations"][station][newProperty] = newValue
                conf["workflow"]["timestamp"] = datetime.now().strftime(
                    "%m/%d/%Y, %H:%M:%S")
                db.workflowConfig.replace_one(
                    {"workflow": {"$exists": True}}, conf)
                print("Success: Property " + str(newProperty) + " in station " + str(station) +
                      " successfully changed from " + str(oldValue) + " to " + str(newValue))
                return True
            else:
                print("Error: Property not found in station")
                return False
        else:
            print("Error: Station not found in config")
            return False

    def changeLiquidProperty(self, liquid: str, newProperty, newValue):
        db = self.client.config
        conf = db.workflowConfig.find_one({"workflow": {"$exists": True}})
        if (liquid in conf["workflow"]["Materials"]["liquids"]):
            if (newProperty in conf["workflow"]["Materials"]["liquids"][liquid]):
                oldValue = conf["workflow"]["Materials"]["liquids"][liquid][newProperty]
                conf["workflow"]["Materials"]["liquids"][liquid][newProperty] = newValue
                conf["workflow"]["timestamp"] = datetime.now().strftime(
                    "%m/%d/%Y, %H:%M:%S")
                db.workflowConfig.replace_one(
                    {"workflow": {"$exists": True}}, conf)
                print("Success: Property " + str(newProperty) + " in liquid " + str(liquid) +
                      " successfully changed from " + str(oldValue) + " to " + str(newValue))
                return True
            else:
                print("Error: Property not found in station")
                return False
        else:
            print("Error: Station not found in config")
            return False

    def changeSolidProperty(self, solid: str, newProperty, newValue):
        db = self.client.config
        conf = db.workflowConfig.find_one({"workflow": {"$exists": True}})
        if (solid in conf["workflow"]["Materials"]["solids"]):
            if (newProperty in conf["workflow"]["Materials"]["solids"][solid]):
                oldValue = conf["workflow"]["Materials"]["solids"][solid][newProperty]
                conf["workflow"]["Materials"]["solids"][solid][newProperty] = newValue
                conf["workflow"]["timestamp"] = datetime.now().strftime(
                    "%m/%d/%Y, %H:%M:%S")
                db.workflowConfig.replace_one(
                    {"workflow": {"$exists": True}}, conf)
                print("Success: Property " + str(newProperty) + " in solid " + str(solid) +
                      " successfully changed from " + str(oldValue) + " to " + str(newValue))
                return True
            else:
                print("Error: Property not found in station")
                return False
        else:
            print("Error: Station not found in config")
            return False

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
