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


    def advanceSuccess(self):
        nextNode = self.currentNode.onsuccess
        for node in self.nodes:
            if (nextNode == node.nodename):
                self.currentNode = node
                return
    
    def advanceFail(self):
        nextNode = self.currentNode.onfail
        for node in self.nodes:
            if (nextNode == node.nodename):
                self.currentNode = node
                return

    def hasEnded(self):
        return self.currentNode.nodename == 'end'

class Recipe:
    def __init__(self, name: str, id: int, stationOpDescriptors: List[StationOpDescriptor], stationFlow: StationFlow, solids: List[Solid], liquids: List[Liquid]):
        self._name = name
        self._id = id
        self._stationopDescriptors = stationOpDescriptors
        self._solids = solids
        self._liquids = liquids
        self._stationFlow = stationFlow

    @property
    def name(self):
        return self._name

    @property
    def id(self):
        return self._id

    @property
    def stationOpDescriptors (self):
        return self.stationopDescriptors

    @property
    def solids (self):
        return self._solids

    @property
    def liquids (self):
        return self._liquids

    @property
    def stationFlow (self):
        return self._stationFlow
