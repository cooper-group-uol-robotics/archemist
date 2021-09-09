from archemist.state.material import Liquid, Solid
from archemist.state.station import Station, StationOpDescriptor
from typing import List

class StationFlowNode:
    def __init__(self, node: int, station: Station, task: list, onsuccess: int, onfail: int):
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
        if (self.nodes[self.currentNodeID-1].station.getResult().success):
            self.currentNodeID = self.currentNode.onsuccess
            self.currentNode = self.nodes[self.currentNodeID-1]
        else:
            self.currentNodeID =self.currentNode.onfail
            self.currentNode = self.nodes[self.currentNodeID-1]
        return self.currentNode

    def __len__(self):
        return len(self.nodes)

class Recipe:
    def __init__(self, name: str, id: int, stationOpDescriptors: List[StationOpDescriptor], stationFlow: StationFlow, solids: List[Solid], liquids: List[Liquid]):
        self._name = name
        self._id = id
        self.stationopDescriptors = stationOpDescriptors
        self.solids = solids
        self.liquids = liquids
        self.stationFlow = stationFlow

    @property
    def name(self):
        return self._name

    @property
    def id(self):
        return self._id
