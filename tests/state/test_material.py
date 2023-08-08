import unittest
from mongoengine import connect,errors
from archemist.core.state.material import Liquid, Material, Solid
from datetime import date, timedelta


class MaterialTest(unittest.TestCase):

    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

    def tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()  

    def test_liquid(self):
        liquid_dict = {
            'name': 'water',
            'id': 1254,
            'amount': 400,
            'unit': 'mL',
            'density': 997,
            'density_unit': 'kg/m3',
            'expiry_date': date.fromisoformat('2025-02-11'),
            'details': {
                'pump_id': 'pUmP1'
                }
        }
        # construct liquid
        t_liquid = Liquid.from_dict(liquid_dict)
        self.assertEqual(t_liquid.name, 'water')
        self.assertEqual(t_liquid.id, 1254)
        self.assertEqual(t_liquid.density, 997)
        self.assertEqual(t_liquid.density_unit, "kg/m3")
        self.assertEqual(t_liquid.volume, 400)
        self.assertEqual(t_liquid.mass, 997*0.4)
        self.assertEqual(t_liquid.details["pump_id"], 'pUmP1')
        self.assertEqual(t_liquid.expiry_date, date.fromisoformat('2025-02-11'))
        
        # increase volume
        t_liquid.increase_volume(1, 'L')
        self.assertEqual(t_liquid.volume, 1400)
        self.assertEqual(t_liquid.mass, 997*1.4)
        
        # decrease volume
        t_liquid.decrease_volume(1.3, 'L')
        self.assertAlmostEqual(t_liquid.volume, 100, 6)
        self.assertAlmostEqual(t_liquid.mass, 997*0.1, 6)
        with self.assertRaises(errors.ValidationError):
            t_liquid.decrease_volume(1.3, 'L')
    
    def test_solid(self):
        solid_dict = {
            'name': 'sodium_chloride',
            'id': 133,
            'amount': 10000,
            'unit': 'ug',
            'expiry_date': date.fromisoformat('2025-02-11'),
            'details': {'dispense_src': 'quantos', 'cartridge_id': 123}
        }

        t_solid = Solid.from_dict(solid_dict)
        self.assertEqual(t_solid.name, 'sodium_chloride')
        self.assertEqual(t_solid.id, 133)
        self.assertEqual(t_solid.mass, 10000)
        self.assertEqual(t_solid.mass_unit, "ug")
        self.assertEqual(t_solid.expiry_date, date.fromisoformat('2025-02-11'))
        self.assertEqual(len(t_solid.details), 2)
        self.assertEqual(t_solid.details['dispense_src'], 'quantos')
        self.assertEqual(t_solid.details["cartridge_id"], 123)

        # increase mass
        t_solid.increase_mass(1, "mg")
        self.assertAlmostEqual(t_solid.mass, 11000, 6)

        # decrease mass
        t_solid.decrease_mass(5000, "ug")
        self.assertAlmostEqual(t_solid.mass, 6000, 6)
        with self.assertRaises(errors.ValidationError):
            t_solid.decrease_mass(55000, "ug")

if __name__ == '__main__':
    connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    unittest.main()
