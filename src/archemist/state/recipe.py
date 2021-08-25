class Recipe:
    def __init__(self, name, id, stationFlow):
        self.name = name
        self.id = id
        self.stationDescriptors = list()
        self.materials = list()
        self.stationFlow = stationFlow
        self.outcomeDescriptors = list()
    
class StationDescriptor:
    def __init__(self, name, stationAssoc, id):
        self.name = name
        self.id = id
        self.station = stationAssoc
        self.tasks = list()

class StationFlow:
    def __init__(self, node, station, task, outcome, onsuccess, onfail):
        self.stateTable = dict()
        self.node = node
        self.station = station
        self.task = task
        self.outcome = outcome
        self.onsuccess = onsuccess
        self.onfail = onfail

        
