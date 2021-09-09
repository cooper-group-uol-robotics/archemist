import unittest

from archemist.state.recipe import Recipe, StationFlow, StationFlowNode, StationOpDescriptor
from archemist.state.station import Station, Location
from archemist.state.result import Result
from archemist.state.material import Liquid, Solid
from datetime import date

class RecipeTest(unittest.TestCase):
    
    def test_recipe(self):
        t_location = Location(name="newLoc", node_id=2, graph_id=3, map_id=4)
        t_station = Station(name="newStation", id=1, location=t_location)
        t_result = Result("newresult", "characteristic", bool)
        t_stationflownode1 = StationFlowNode(1, t_station, "dosomething", 2, 1)
        t_stationflownode2 = StationFlowNode(2, t_station, "dosomethingalso", 3, 2)
        t_stationflownode3 = StationFlowNode(3, t_station, "dosomethingalsothen", 0, 3)
        t_stationflow = StationFlow([t_stationflownode1, t_stationflownode2, t_stationflownode3])
        t_stationdescriptor1 = StationOpDescriptor("dosomething", t_station, 5)
        t_stationdescriptor2 = StationOpDescriptor("dosomethingalso", t_station, 6)
        t_stationdescriptor3 = StationOpDescriptor("dosomethingalsothen", t_station, 7)
        t_stationdescriptors = [t_stationdescriptor1, t_stationdescriptor2, t_stationdescriptor3]
        t_solid = Solid(name='some_solid', id=3333,
                        expiry_date=date.today(), mass=5.0,
                        dispense_method='quantos')
        t_liquid = Liquid(name='some_liquid', id=2222,
                          expiry_date=date.today(), mass=1.0,
                          density=1.0, volume=1.0)
        t_recipe = Recipe("newRecipe", 23, t_stationdescriptors, t_stationflow, [t_solid], [t_liquid], [t_result])

        self.assertEqual(t_recipe.name, "newRecipe")
        self.assertEqual(t_recipe.id, 23)

        self.assertEqual(t_recipe.stationDescriptors[0].name, "dosomething")
        self.assertEqual(t_recipe.stationDescriptors[1].name, "dosomethingalso")
        self.assertEqual(t_recipe.stationDescriptors[2].name, "dosomethingalsothen")

        self.assertEqual(t_recipe.stationFlow.currentNode.task, "dosomething")
        self.assertEqual(t_recipe.stationFlow.currentNode.station.location.node_id, 2)

        self.assertEqual(t_recipe.outcomeDescriptors[0].cType, bool)

        self.assertEqual(t_recipe.liquids[0].name, "some_liquid")
        self.assertEqual(t_recipe.solids[0].name, "some_solid")

    def test_stationflow(self):
        t_location = Location(name="newLoc", node_id=2, graph_id=3, map_id=4)
        t_station = Station(name="newStation", id=1, location=t_location)
        t_stationflownode1 = StationFlowNode(1, t_station, "dosomething", 2, 1)
        t_stationflownode2 = StationFlowNode(2, t_station, "dosomethingalso", 3, 2)
        t_stationflownode3 = StationFlowNode(3, t_station, "dosomethingalsothen", 0, 3)
        t_stationflow = StationFlow([t_stationflownode1, t_stationflownode2, t_stationflownode3])

        self.assertEqual(len(t_stationflow), 3)
        self.assertEqual(t_stationflow.currentNode, t_stationflownode1)
        self.assertEqual(t_stationflow.currentNodeID, 1)
        self.assertEqual(t_stationflow.nextNode(), t_stationflownode2)
        self.assertEqual(t_stationflow.currentNodeID, 2)

        self.assertEqual(t_stationflow.currentNode.task, "dosomethingalso")
        self.assertEqual(t_stationflow.currentNode.station.location.node_id, 2)

    def test_stationflownode(self):
        t_location = Location(name="newLoc", node_id=2, graph_id=3, map_id=4)
        t_station = Station(name="newStation", id=1, location=t_location)
        t_stationflownode = StationFlowNode(1, t_station, "dosomething", 2, 1)

        self.assertEqual(t_stationflownode.nodeid, 1)
        self.assertEqual(t_stationflownode.onsuccess, 2)
        self.assertEqual(t_stationflownode.onfail, 1)
        self.assertEqual(t_stationflownode.task, "dosomething") 
        self.assertEqual(t_stationflownode.station, t_station)

        self.assertEqual(t_stationflownode.station.location.node_id, 2)

    def test_stationdescriptor(self):
        t_location = Location(name="newLoc", node_id=2, graph_id=3, map_id=4)
        t_station = Station(name="newStation", id=1, location=t_location)
        t_stationdescriptor = StationOpDescriptor("taskDescriptor", t_station, 5)

        with self.assertRaises(AttributeError):
            t_stationdescriptor.name = "beep"

        with self.assertRaises(AttributeError):
            t_stationdescriptor.id = 5

        with self.assertRaises(AttributeError):
            t_station2 = Station(name="another", id=2, location=t_location)
            t_stationdescriptor.station = t_station2

        self.assertEqual(t_stationdescriptor.station, t_station)
        self.assertEqual(t_stationdescriptor.name, "taskDescriptor")
        self.assertEqual(t_stationdescriptor.id, 5)
        self.assertEqual(t_stationdescriptor.station.location.node_id, 2)





if __name__ == '__main__':
    unittest.main()

