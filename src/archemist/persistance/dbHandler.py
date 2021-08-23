from pymongo import MongoClient
import persistance.fsHandler
import os
class dbHandler:
    def __init__(self):
        self.client = MongoClient("mongodb://localhost:27017")
        self.db=self.client.admin
        print("Connected, Host: " + self.db.command("serverStatus")["host"])
    
    def importConfig(self):
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

    def importRecipe(self):
        __location__ = os.path.realpath(
        os.path.join(os.getcwd(), os.path.dirname(__file__)))
        handler = persistance.fsHandler.FSHandler()
        db = self.client.config
        if (db.currentRecipe.count_documents({"workflow": {"$exists": True}}) > 0):
            db.currentRecipe.replace_one({"workflow": {"$exists": True}}, handler.loadYamlFile(os.path.join(__location__, 'recipe.yaml')))
            return True
        else:
            db.currentRecipe.insert_one(handler.loadYamlFile(os.path.join(__location__, 'config.yaml')))
            return False

    def getConfig(self):
        db = self.client.config
        return db.workflowConfig.find_one({"workflow": {"$exists": True}})
    
    def getCurrentRecipe(self):
        db = self.client.config
        return db.currentRecipe.find_one({"workflow": {"$exists": True}})




