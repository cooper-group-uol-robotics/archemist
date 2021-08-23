import persistence.dbHandler
import persistence.fsHandler
import persistence.yParser

class persistenceManager:
    def __init__(self):
        self.dbhandler = persistence.dbHandler.dbHandler()
        self.fshandler = persistence.fsHandler.FSHandler()
        self.parser = persistence.yParser.Parser()