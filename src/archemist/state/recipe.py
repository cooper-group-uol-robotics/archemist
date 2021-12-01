from archemist.state.material import Liquid, Solid
from archemist.state.station import Station, StationOpDescriptor
from typing import List

class StationFlowNode:
    def __init__(self, node: str, station: Station, task: list, onsuccess: str, onfail: str):
        self.node_name = node
        self.station = station
        self.task = task
        self.on_success = onsuccess
        self.on_fail = onfail

class StationFlow:
    def __init__(self, stationFlowNodes: List[StationFlowNode]):
        self.nodes = stationFlowNodes
        self.current_node = self.nodes[0]

    def advance_node_success(self):
        nextNode = self.current_node.on_success
        for node in self.nodes:
            if (nextNode == node.node_name):
                self.current_node = node
                return
    
    def advance_node_fail(self):
        nextNode = self.current_node.on_fail
        for node in self.nodes:
            if (nextNode == node.node_name):
                self.current_node = node
                return

    def has_ended(self):
        return self.current_node.node_name == 'end'

    def get_upcoming_node(self):
        nextNode = self.current_node.on_success
        for node in self.nodes:
            if (nextNode == node.node_name):
                return node

class Recipe:
    def __init__(self, name: str, id: int, stationOpDescriptors: List[StationOpDescriptor], stationFlow: StationFlow, solids: List[Solid], liquids: List[Liquid]):
        self._name = name
        self._id = id
        self._station_op_descriptors = stationOpDescriptors
        self._solids = solids
        self._liquids = liquids
        self._station_flow = stationFlow

    @property
    def name(self):
        return self._name

    @property
    def id(self):
        return self._id

    @property
    def station_op_descriptors (self):
        return self._station_op_descriptors

    @property
    def solids (self):
        return self._solids

    @property
    def liquids (self):
        return self._liquids

    @property
    def station_flow (self):
        return self._station_flow

    def get_current_recipe_state(self):
        return self._station_flow.current_node

    def advance_recipe_state(self, success: bool):
        if success:
            self._station_flow.advance_node_success()
        else:
            self._station_flow.advance_node_fail()
        self._logRecipe('Current state advanced to ' + self.station_flow.current_node.node_name)

    def is_complete(self):
        return self._station_flow.has_ended()

    def get_current_task_op(self):
        for taskOp in self._station_op_descriptors:
            if taskOp.__class__.__name__ == self._station_flow.current_node.task:
                return taskOp

    def _logRecipe(self, message: str):
        print(f'Recipe [{self._id}]: ' + message)


