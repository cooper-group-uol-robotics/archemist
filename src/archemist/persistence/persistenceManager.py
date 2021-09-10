from archemist.persistence.dbHandler import dbHandler
from archemist.persistence.fsHandler import FSHandler
from archemist.persistence.yParser import Parser
import os
from datetime import datetime
from bson.binary import Binary
from bson.objectid import ObjectId
import pickle



class persistenceManager:
    def __init__(self):
        self.dbhandler = dbHandler()
        self.fshandler = FSHandler()
        self.parser = Parser()
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
        return self.parser.loadConfigYaml(config)

    def overwriteConfig(self):
        __location__ = os.path.realpath(
            os.path.join(os.getcwd(), os.path.dirname(__file__)))
        config = dict()
        config = {'workflow': self.dbhandler.getConfig()}
        self.fshandler.overwriteYamlFile(config, os.path.join(
            __location__, 'workflowConfigs/config.yaml'))
        self.loadedConfig = config
        return self.parser.loadConfigYaml(config)

    def storeWorkFlowState(self, state):
        client = self.dbhandler.getDBAccess()
        db = client['test']
        state_coll = db.test_collection
        flatDocument = dict()
        for station in state._stations:
            flatDocument[station.__class__.__name__] = Binary(pickle.dumps(station))
        for robot in state._robots:
            flatDocument[robot.__class__.__name__] = Binary(pickle.dumps(robot))
        for liquid in state._liquids:
            flatDocument[liquid.name] = Binary(pickle.dumps(liquid))
        for solid in state._solids:
            flatDocument[solid.name] = Binary(pickle.dumps(solid))
        return state_coll.insert(flatDocument)

    def retrieveWorkflowState(self):
        client = self.dbhandler.getDBAccess()
        db = client['test']
        state_coll = db.test_collection
        statess = state_coll.find_one({'_id': ObjectId('613ad79b370a233f6aa5d7f6')})
        return statess

    def updateObjectState(self, object):
        client = self.dbhandler.getDBAccess()
        db = client['test']
        state_coll = db.test_collection
        new_pickled_obj = Binary(pickle.dumps(object))
        state_coll.update({'_id': ObjectId('613ad79b370a233f6aa5d7f6')}, {'$set': {object.__class__.__name__, new_pickled_obj}})





