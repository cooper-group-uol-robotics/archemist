from pymongo import MongoClient
import persistence.fsHandler
import os
from datetime import datetime
class dbHandler:
    def __init__(self):
        self.client = MongoClient("mongodb://localhost:27017")
        self.db=self.client.admin
        print("Connected, Host: " + self.db.command("serverStatus")["host"])

    def changeStationProperty(self, station: str, newProperty, newValue):
        db = self.client.config
        conf = db.workflowConfig.find_one({"workflow": {"$exists": True}})
        if (station in conf["workflow"]["Stations"]):
            if (newProperty in conf["workflow"]["Stations"][station]):
                oldValue =  conf["workflow"]["Stations"][station][newProperty]
                conf["workflow"]["Stations"][station][newProperty] = newValue
                db.workflowConfig.replace_one({"workflow": {"$exists": True}}, conf)
                print("Success: Property " + str(newProperty) + " in station " + str(station) + " successfully changed from " + str(oldValue) + " to "+ str(newValue))
                return True
            else:
                print("Error: Property not found in station")
                return False
        else:
            print("Error: Station not found in config")
            return False

    def importConfig(self):
        __location__ = os.path.realpath(
        os.path.join(os.getcwd(), os.path.dirname(__file__)))
        handler = persistence.fsHandler.FSHandler()
        db = self.client.config
        conf = handler.loadYamlFile(os.path.join(__location__, 'workflowConfigs\config.yaml'))
        conf["workflow"]["timestamp"] = datetime.now().strftime("%m/%d/%Y, %H:%M:%S")
        if (db.workflowConfig.count_documents({"workflow": {"$exists": True}}) > 0):
            db.workflowConfig.replace_one({"workflow": {"$exists": True}}, conf)
            return True
        else:
            db.workflowConfig.insert_one(conf)
            return False

    def importRecipe(self):
        __location__ = os.path.realpath(
        os.path.join(os.getcwd(), os.path.dirname(__file__)))
        handler = persistence.fsHandler.FSHandler()
        db = self.client.config
        if (db.currentRecipe.count_documents({"workflow": {"$exists": True}}) > 0):
            db.currentRecipe.replace_one({"workflow": {"$exists": True}}, handler.loadYamlFile(os.path.join(__location__, 'workflowConfigs\recipes\recipe.yaml')))
            return True
        else:
            db.currentRecipe.insert_one(handler.loadYamlFile(os.path.join(__location__, 'workflowConfigs\config.yaml')))
            return False

    def getConfig(self):
        db = self.client.config
        return db.workflowConfig.find_one({"workflow": {"$exists": True}})["workflow"]
    
    def getCurrentRecipe(self):
        db = self.client.config
        return db.currentRecipe.find_one({"workflow": {"$exists": True}})




