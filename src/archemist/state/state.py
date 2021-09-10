from archemist.persistence.persistenceManager import persistenceManager
from archemist.persistence.yParser import Parser
from datetime import datetime


class State:
    def __init__(self):
        self.startTime = datetime.now()
        self.liquids = []
        self.solids = []
        self.stations = []
        self.robots = []
    
    def initializeState(self):
        self.persistence = persistenceManager()
        parser = Parser()
        config = self.persistence.loadConfig()
        self.robots = config[0]
        self.liquids = config[1]
        self.solids = config[2]
        self.stations = config[3]
        self.recipe = parser.loadRecipeYaml()

    def store(self):
        self.persistence.storeWorkFlowState(self)

    def update(self):
        self.persistence.retrieveWorkflowState()


        