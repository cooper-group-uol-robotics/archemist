import unittest

from archemist.core.util.location import Location


class LocationTest(unittest.TestCase):

    def test_location(self):
        location_1 = Location.from_args(coordinates=(1, 2), descriptor="InputSite")
        self.assertEqual(location_1.coordinates, (1, 2))
        self.assertEqual(location_1.descriptor, "InputSite")
        self.assertFalse(location_1.is_unspecified())

        location_2 = Location.from_dict({"coordinates": [1, 2], "descriptor": "InputSite"})
        self.assertEqual(location_2.coordinates, (1, 2))
        self.assertEqual(location_2.descriptor, "InputSite")

        self.assertEqual(location_1, location_2)
        self.assertEqual(str(location_1), f"coordinates:{location_1.coordinates} - descriptor: {location_1.descriptor}")

        location_3 = Location.from_args()
        self.assertTrue(location_3.is_unspecified())
        self.assertNotEqual(location_1, location_3)


if __name__ == "__main__":
    unittest.main()
