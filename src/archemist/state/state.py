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
        self.processed_batches = []
    
    def initializeState(self, reset_db: bool):
        self.persistence = persistenceManager()
        config = self.persistence.loadConfig()
        self.robots = config[0]
        self.liquids = config[1]
        self.solids = config[2]
        self.stations = config[3]
        if (reset_db):
            self.storeToDB()
        else:
            self.updateFromDB()
        #self.recipe = parser.loadRecipeYaml()
        #parser = Parser()

    def getStation(self, stationName: str):
        return next(station for station in self.stations if station.__class__.__name__ == stationName)

    def getRobot(self, name: str, id: int):
        return next(robot for robot in self.robots if (robot.__class__.__name__ == name and robot.id == id))

    def storeToDB(self):
        self.persistence.push(self)

    def updateFromDB(self):
        state_dict = self.persistence.pull()
        updated_stations = [state_dict[station.__class__.__name__] for station in self.stations]
        self.stations = updated_stations
        updated_robots = [state_dict[robot.__class__.__name__] for robot in self.robots]
        self.robots = updated_robots
        updated_solids = [state_dict[solid.name] for solid in self.solids]
        self.solids = updated_solids
        updated_liquids = [state_dict[liquid.name] for liquid in self.liquids]
        self.liquids = updated_liquids
        if 'processed_batches' in state_dict:
            self.processed_batches = state_dict['processed_batches']

    def modifyObjectDB(self, object):
        self.persistence.updateObjectState(object)