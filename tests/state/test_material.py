import unittest

from archemist.state.material import Liquid, Material, Solid
from datetime import date, timedelta


class MaterialTest(unittest.TestCase):

    def test_material(self):
        t_material = Material(name='Some_Material', id=1234,
                              expiry_date=date.today(), mass=1.5)
        self.assertEqual(t_material.name, 'Some_Material')
        with self.assertRaises(AttributeError):
            t_material.name = 'stupid_name'
        self.assertEqual(t_material.id, 1234)
        with self.assertRaises(AttributeError):
            t_material.id = 6789
        self.assertEqual(t_material.expiry_date, date.today())
        with self.assertRaises(AttributeError):
            t_material.expiry_date = date.today() - timedelta(days=1)
        self.assertEqual(t_material.mass, 1.5)
        t_material.mass = 2.0
        self.assertEqual(t_material.mass, 2.0)
        with self.assertRaises(ValueError):
            t_material.mass = -2.0
        self.assertEqual(t_material.mass, 2.0)

    def test_liquid(self):
        t_liquid = Liquid(name='some_liquid', id=2222,
                          expiry_date=date.today(), mass=1.0,
                          density=1.0, volume=1.0)
        self.assertEqual(t_liquid.density, 1)
        t_liquid.density = 1.1
        self.assertEqual(t_liquid.density, 1.1)
        with self.assertRaises(ValueError):
            t_liquid.density = -2.0
        self.assertEqual(t_liquid.volume, 1.0)
        t_liquid.volume = 1.1
        self.assertEqual(t_liquid.volume, 1.1)
        with self.assertRaises(ValueError):
            t_liquid.volume = -2.0
        t_liquid_str = str(t_liquid)
        self.assertEqual(t_liquid_str, 'Liquid: {0}, ID: {1}, Expiry date: {2},\
                 Mass: {3} g, Volume: {4} L,\
                 Density: {5} g/L'.format(t_liquid.name,
                        t_liquid.id, t_liquid.expiry_date, t_liquid.mass,
                        t_liquid.volume, t_liquid.density))

    def test_solid(self):
        t_solid = Solid(name='some_solid', id=3333,
                        expiry_date=date.today(), mass=5.0,
                        dispense_method='quantos')
        self.assertEqual(t_solid.dispense_method, 'quantos')
        t_solid.dispense_method = 'new_method'
        self.assertEqual(t_solid.dispense_method, 'new_method')
        t_solid_str = str(t_solid)
        self.assertEqual(t_solid_str, 'Solid: {0}, ID: {1}, Expiry date: {2},\
                 Mass: {3} g, Dispense method: {4}'.format(t_solid.name,
                        t_solid.id, t_solid.expiry_date, t_solid.mass,
                        t_solid.dispense_method))


if __name__ == '__main__':
    unittest.main()
