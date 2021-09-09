from archemist.state.material import Liquid, Solid
from archemist.state.station import Station, StationOpDescriptor
from typing import List

class StationFlowNode:
    def __init__(self, node: str, station: Station, task: list, onsuccess: str, onfail: str):
        self.nodename = node
        self.station = station
        self.task = task
        self.onsuccess = onsuccess
        self.onfail = onfail

class StationFlow:
    def __init__(self, stationFlowNodes: List[StationFlowNode]):
        self.nodes = stationFlowNodes
        self.currentNode = self.nodes[0]

    def nextNode(self):
        if (self.currentNode.station == None):
            nextNodeStr = self.currentNode.onsuccess
            for node in self.nodes:
                if (node.nodename == nextNodeStr):
                    self.nextNode = node
            self.currentNode = self.nextNode
        elif(self.currentNode.station.getOperationResult() == None):    
            nextNodeStr = self.currentNode.onsuccess
            for node in self.nodes:
                if (node.nodename == nextNodeStr):
                    self.nextNode = node
            self.currentNode = self.nextNode
        else:
            if (self.currentNode.station.getOperationResult().success()):
                nextNodeStr = self.currentNode.onsuccess
                for node in self.nodes:
                    if (node.nodename == nextNodeStr):
                        self.nextNode = node
                self.currentNode = self.nextNode
            elif(not self.currentNode.station.getOperationResult().success()):
                nextNodeStr = self.currentNode.onfail
                for node in self.nodes:
                    if (node.nodename == nextNodeStr):
                        self.nextNode = node
                self.currentNode = self.nextNode
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
