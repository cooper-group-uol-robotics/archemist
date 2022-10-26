import unittest
from mongoengine import connect,errors
from archemist.core.state.material import Liquid, Material, Solid
from datetime import date, timedelta


class MaterialTest(unittest.TestCase):

    def test_liquid(self):
        liquid_dict = {
            'name': 'water',
            'id': 1254,
            'amount_stored': 400,
            'unit': 'ml',
            'density': 997,
            'pump_id': 'pUmP1',
            'expiry_date': date.fromisoformat('2025-02-11')
        }

        t_liquid = Liquid.from_dict(liquid_dict)
        self.assertEqual(t_liquid.name, 'water')
        self.assertEqual(t_liquid.id, 1254)
        self.assertEqual(t_liquid.density, 997)
        self.assertEqual(t_liquid.volume, 0.4)
        self.assertEqual(t_liquid.mass, 997*0.4)
        t_liquid.volume = 1.1
        self.assertEqual(t_liquid.volume, 1.1)
        with self.assertRaises(errors.ValidationError):
            t_liquid.volume = -2.0
        self.assertEqual(t_liquid.pump_id, 'pUmP1')
        self.assertEqual(t_liquid.expiry_date, date.fromisoformat('2025-02-11'))
    
    def test_solid(self):
        solid_dict = {
            'name': 'sodium_chloride',
            'id': 133,
            'amount_stored': 10000,
            'dispense_src': 'quantos',
            'cartridge_id': 123,
            'unit': 'ug',
            'expiry_date': date.fromisoformat('2025-02-11')
        }

        t_solid = Solid.from_dict(solid_dict)
        self.assertEqual(t_solid.name, 'sodium_chloride')
        self.assertEqual(t_solid.id, 133)
        self.assertEqual(t_solid.mass, 0.01)
        t_solid.mass = 0.1
        self.assertEqual(t_solid.mass, 0.1)
        with self.assertRaises(errors.ValidationError):
            t_solid.mass = -2.0
        self.assertEqual(t_solid.dispense_src, 'quantos')
        self.assertEqual(t_solid.expiry_date, date.fromisoformat('2025-02-11'))

if __name__ == '__main__':
    connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    unittest.main()
