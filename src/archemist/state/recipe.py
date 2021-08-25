class StationDescriptor:
    def __init__(self, name, stationAssoc, id):
        self.name = name
        self.id = id
        self.station = stationAssoc

class StationFlowNode:
    def __init__(self, node, station, task, outcome, onsuccess, onfail):
        self.nodeid = node
        self.station = station
        self.task = task
        self.outcome = outcome
        self.onsuccess = onsuccess
        self.onfail = onfail

class StationFlow:
    def __init__(self, stationFlowNodes: list):
        self.nodes = stationFlowNodes
        self.currentNode = 1
        self.nextNode = None 

class Recipe:
    def __init__(self, name, id, stationFlow: StationFlow, solids: list, liquids:list):
        self.name = name
        self.id = id
        self.stationDescriptors = list()
        self.solids = solids
        self.liquids = liquids
        self.stationFlow = stationFlow
        self.outcomeDescriptors = list()
    

