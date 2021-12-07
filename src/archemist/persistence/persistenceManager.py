from archemist.persistence.dbHandler import dbHandler
from archemist.persistence.fsHandler import FSHandler
from archemist.persistence.yParser import Parser
import os
from datetime import datetime
import pickle
import codecs
from bson import ObjectId

from archemist.state.material import Liquid, Solid

class persistenceManager:
    def __init__(self):
        self.dbhandler = dbHandler()
        self.fshandler = FSHandler()
        self.parser = Parser()
        self.loadedConfig = None
        self.client = self.dbhandler.getDBAccess()
        self.lastObject = None

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
            __location__, 'workflowConfigs/config_test.yaml'))
        self.loadedConfig = config
        return self.parser.loadConfigYaml(config)

    def push(self, state):
        client = self.dbhandler.getDBAccess()
        db = client['test']
        state_coll = db.test_collection
        flatDocument = dict()
        for station in state.stations:
            flatDocument[station.__class__.__name__] = codecs.encode(pickle.dumps(station), "base64").decode()
        for robot in state.robots:
            flatDocument[robot.__class__.__name__] = codecs.encode(pickle.dumps(robot), "base64").decode()
        for liquid in state.liquids:
            flatDocument[liquid.name] = codecs.encode(pickle.dumps(liquid), "base64").decode()
        for solid in state.solids:
            flatDocument[solid.name] = codecs.encode(pickle.dumps(solid), "base64").decode()
        if state.processed_batches:
            flatDocument['processed_batches'] = codecs.encode(pickle.dumps(state.processed_batches), "base64").decode()
        self.lastObject = state_coll.replace_one({'_id': ObjectId('613b2fb9df96390d3263f0e4')}, flatDocument, upsert=True)
        return self.lastObject

    # def pull(self):
    #     client = self.dbhandler.getDBAccess()
    #     db = client['test']
    #     state_coll = db.test_collection
    #     statess = state_coll.find_one({'_id': ObjectId('613b2fb9df96390d3263f0e4')})
    #     statesCol = dict()
    #     for state in statess:
    #         if (state != "_id"):
    #             statesCol[state] = pickle.loads(codecs.decode(statess[state].encode(), "base64"))
    #     return statesCol
    
    def pull(self):
        client = self.dbhandler.getDBAccess()
        db = client['test']
        state_coll = db.test_collection
        statess = state_coll.find_one({'_id': ObjectId('613b2fb9df96390d3263f0e4')})
        statesCol = dict()
        for state in statess:
            if (state != "_id"):
                statesCol[state] = pickle.loads(codecs.decode(statess[state].encode(), "base64"))
        return statesCol

    def updateObjectState(self, object):
        client = self.dbhandler.getDBAccess()
        db = client['test']
        state_coll = db.test_collection
        new_pickled_obj = codecs.encode(pickle.dumps(object), "base64").decode()
        if (isinstance(object, Liquid) or isinstance(object, Solid)):
            state_coll.update_one({'_id': ObjectId('613b2fb9df96390d3263f0e4')}, {'$set': {object.name: new_pickled_obj}})
        else:
            state_coll.update_one({'_id': ObjectId('613b2fb9df96390d3263f0e4')}, {'$set': {object.__class__.__name__: new_pickled_obj}})
