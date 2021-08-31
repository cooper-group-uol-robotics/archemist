import unittest
from src.archemist.exceptions.exception import StationAssignedRackError, StationNoOutcomeError, StationUnAssignedRackError

from src.archemist.state.station import Location, Station, SolidDispensingStation
from src.archemist.state.batch import Batch

class StationTest(unittest.TestCase):

    def test_location(self):
        t_location = Location(name="newLoc", node_id=2, graph_id=3, map_id=4)
        self.assertEqual(t_location.name, "newLoc")
        with self.assertRaises(AttributeError):
            t_location.name="somenewname"
        self.assertEqual(t_location.node_id, 2)
        with self.assertRaises(AttributeError):
            t_location.node_id=10
        self.assertEqual(t_location.graph_id, 3)
        with self.assertRaises(AttributeError):
            t_location.graph_id=10
        self.assertEqual(t_location.map_id, 4)
        with self.assertRaises(AttributeError):
            t_location.map_id=10

    def test_station(self):
        t_location = Location(name="newLoc", node_id=2, graph_id=3, map_id=4)
        t_station = Station(name="newStation", id=1, location=t_location)

        self.assertEqual(t_station.name, "newStation")
        with self.assertRaises(AttributeError):
            t_station.name="somenewname"

        self.assertEqual(t_station.id, 1)
        with self.assertRaises(AttributeError):
            t_station.id=2

        self.assertEqual(t_station.location, t_location)
        t_location_2 = Location(name="anotherLoc", node_id=1, graph_id=1, map_id=1)
        with self.assertRaises(AttributeError):
            t_station.location = t_location_2
        self.assertEqual(t_station.location.name, "newLoc")
        self.assertEqual(t_station.location.node_id, 2)
        self.assertEqual(t_station.location.graph_id, 3)
        self.assertEqual(t_station.location.map_id, 4)

        self.assertEqual(t_station.available, False)
        with self.assertRaises(ValueError):
            t_station.available = 6
        t_station.available = True
        self.assertEqual(t_station.available, True)
        self.assertEqual(t_station.operational, False)
        with self.assertRaises(ValueError):
            t_station.operational = 6
        t_station.operational = True
        self.assertEqual(t_station.operational, True)

        t_batch = Batch(name="newBatch", id=5)
        t_station.add_batch(t_batch)
        self.assertEqual(t_station.retrieve_batch(), t_batch)
        t_station.add_batch(t_batch)
        with self.assertRaises(StationAssignedRackError):
            t_station.add_batch(t_batch)
        t_station.retrieve_batch()
        with self.assertRaises(StationUnAssignedRackError):
            t_station.retrieve_batch()
        
        self.assertEqual(t_station.type, "none")
        t_station_new = SolidDispensingStation(name="newStation", id=1, location=t_location)
        self.assertEqual(t_station_new.type, "SolidDisp")
        with self.assertRaises(AttributeError):
            t_station_new.type="somenewtype"
        self.assertEqual(t_station_new.location.graph_id, 3)

        self.assertEqual(t_station.outcome, True)
        t_station.outcome = False
        self.assertEqual(t_station.outcome, False)
        t_station.finished = False
        with self.assertRaises(StationNoOutcomeError):
            print(t_station.outcome)


if __name__ == '__main__':
    unittest.main()