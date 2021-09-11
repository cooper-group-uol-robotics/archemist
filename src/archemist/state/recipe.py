from archemist.state.material import Liquid, Solid
from archemist.state.station import Station, StationOpDescriptor
from typing import List

class StationFlowNode:
    def __init__(self, node: str, station: Station, task: list, onsuccess: str, onfail: str):
        self.nodename = node
        self.station = station
        self.task = task
        self.pre_load_success = False
        self.load_success = False
        self.processing_success = False
        self.post_load_success = False
        self.onsuccess = onsuccess
        self.onfail = onfail

class StationFlow:
    def __init__(self, stationFlowNodes: List[StationFlowNode]):
        self.nodes = stationFlowNodes
        self.currentNode = self.nodes[0]


    def advanceSuccess(self):
        if not self.currentNode.pre_load_success:
            self.currentNode.pre_load_success = True
            return
        elif not self.currentNode.load_success:
            self.currentNode.load_success = True
            return
        elif not self.currentNode.processing_success:
            self.currentNode.processing_success = True
            return
        elif not self.currentNode.post_load_success:
            self.currentNode.post_load_success = True
            return
        nextNode = self.currentNode.onsuccess
        for node in self.nodes:
            if (nextNode == node.nodename):
                self.currentNode = node
                return

    def advanceNodeSuccess(self):
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

    def reset(self):
        for node in self.nodes:
            node.pre_load_success = False
            node.load_success = False
            node.processing_success = False
            node.post_load_success = False
        self.currentNode = self.nodes[0]

    def getUpcomingNode(self):
        nextNode = self.currentNode.onsuccess
        for node in self.nodes:
            if (nextNode == node.nodename):
                return node

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
        return self._stationopDescriptors

    @property
    def solids (self):
        return self._solids

    @property
    def liquids (self):
        return self._liquids

    @property
    def stationFlow (self):
        return self._stationFlow

    def getCurrentNode(self):
        return self._stationFlow.currentNode

    def advanceState(self, success: bool):
        if success:
            self._stationFlow.advanceSuccess()
        else:
            self._stationFlow.advanceFail()

    def advanceNode(self, success: bool):
        if success:
            self._stationFlow.advanceNodeSuccess()
        else:
            self._stationFlow.advanceFail()

    def resetFlow(self):
        self.stationFlow.reset()

    def hasEnded(self):
        self.stationFlow.hasEnded()

    def getCurrentTaskOp(self):
        for taskOp in self._stationopDescriptors:
            if taskOp.__class__.__name__ == self.stationFlow.currentNode.task:
                return taskOp


