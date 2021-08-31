from src.archemist.state.material import Liquid, Solid
from src.archemist.state.station import Station
from src.archemist.state.result import Result
from typing import List

class StationOpDescriptor:
    def __init__(self, name: str, stationAssoc: Station, id: int):
        self._name = name
        self._id = id
        self._station = stationAssoc
    
    @property
    def name(self):
        return self._name
    
    @property
    def id(self):
        return self._id

    @property
    def station(self):
        return self._station

class StationFlowNode:
    def __init__(self, node: int, station: Station, task: str, onsuccess: int, onfail: int):
        self.nodeid = node
        self.station = station
        self.task = task
        self.onsuccess = onsuccess
        self.onfail = onfail

class StationFlow:
    def __init__(self, stationFlowNodes: List[StationFlowNode]):
        self.nodes = stationFlowNodes
        self.currentNodeID = 1
        self.currentNode = self.nodes[0]

    def nextNode(self):
        if (self.nodes[self.currentNodeID-1].station.outcome):
            self.currentNodeID = self.currentNode.onsuccess
            self.currentNode = self.nodes[self.currentNodeID-1]
        else:
            self.currentNodeID =self.currentNode.onfail
            self.currentNode = self.nodes[self.currentNodeID-1]
        return self.currentNode


    def __len__(self):
        return len(self.nodes)

class Recipe:
    def __init__(self, name: str, id: int, stationFlow: StationFlow, solids: List[Solid], liquids: List[Liquid], outcomeDescriptors: List[Result]):
        self.name = name
        self.id = id
        self.stationDescriptors = list()
        self.solids = solids
        self.liquids = liquids
        self.stationFlow = stationFlow
        self.outcomeDescriptors = outcomeDescriptors
