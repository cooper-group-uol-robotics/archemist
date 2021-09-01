from src.archemist.persistence.persistenceManager import persistenceManager
from src.archemist.persistence.yParser import Parser


class State:
    def __init__(self):

        return True
    
    def initializeState(self):
        persistence = persistenceManager()
        parser = Parser()
        self._config = persistence.loadConfig()
        self._recipe = parser.loadRecipeYaml()

        