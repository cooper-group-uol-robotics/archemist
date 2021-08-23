from pymongo import MongoClient
import persistance.fsHandler
import os
class dbHandler:
    def __init__(self):
        self.client = MongoClient("mongodb://localhost:27017")
        self.db=self.client.admin
        print("Connected, Host: " + self.db.command("serverStatus")["host"])
    
    def syncConfig(self):
        __location__ = os.path.realpath(
        os.path.join(os.getcwd(), os.path.dirname(__file__)))
        handler = persistance.fsHandler.FSHandler()
        db = self.client.config
        if (db.workflowConfig.count_documents({"workflow": {"$exists": True}}) > 0):
            db.workflowConfig.replace_one({"workflow": {"$exists": True}}, handler.loadYamlFile(os.path.join(__location__, 'config.yaml')))
            return True
        else:
            db.workflowConfig.insert_one(handler.loadYamlFile(os.path.join(__location__, 'config.yaml')))
            return False





