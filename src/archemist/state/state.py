from archemist.persistence.persistenceManager import persistenceManager
from archemist.persistence.yParser import Parser
from datetime import datetime

class State:
    def __init__(self):
        self._startTime = datetime.now()
    
    def initializeState(self):
        persistence = persistenceManager()
        parser = Parser()
        self._config = persistence.loadConfig()
        self._recipe = parser.loadRecipeYaml()

        