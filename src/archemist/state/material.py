from datetime import date

from bson.objectid import ObjectId

from archemist.persistence.dbObjProxy import DbObjProxy


class Material(DbObjProxy):
    def __init__(self, db_name: str, material_document: dict):
        
        if len(material_document) > 1:
            material_document['object'] = self.__class__.__name__
            super().__init__(db_name, 'materials', material_document)
        else:
            super().__init__(db_name, 'materials', material_document['object_id'])

    @property
    def name(self):
        return self.get_field('name')

    @property
    def expiry_date(self):
        return self.get_field('expiry_date')

    @property
    def mass(self):
        return self.get_field('mass')

    @mass.setter
    def mass(self, value):
        if value >= 0:
            self.update_field('mass', value)
        else:
            raise ValueError


class Liquid(Material):
    def __init__(self, db_name:str, liquid_document: dict):
        
        if len(liquid_document) > 1:
            if liquid_document['unit'] == 'l':
                liquid_document['volume'] = liquid_document['amount_stored']
                liquid_document['mass'] = liquid_document['density'] * liquid_document['volume']
            elif liquid_document['unit'] == 'ml':
                liquid_document['volume'] = liquid_document['amount_stored']/1000
                liquid_document['mass'] = liquid_document['density'] * liquid_document['volume']
            elif liquid_document['unit'] == 'ul':
                liquid_document['volume'] = liquid_document['amount_stored']/1000000
                liquid_document['mass'] = liquid_document['density'] * liquid_document['volume']
            elif liquid_document['unit'] == 'g':
                liquid_document['mass'] = liquid_document['amount_stored']
                liquid_document['volume'] = liquid_document['mass'] / liquid_document['density']
            elif liquid_document['unit'] == 'mg':
                liquid_document['mass'] = liquid_document['amount_stored']/1000
                liquid_document['volume'] = liquid_document['mass'] / liquid_document['density']
            elif liquid_document['unit'] == 'ug':
                liquid_document['mass'] = liquid_document['amount_stored']/1000000
                liquid_document['volume'] = liquid_document['mass'] / liquid_document['density']
            liquid_document.pop('amount_stored')
        super().__init__(db_name, liquid_document)

    @classmethod
    def from_dict(cls, db_name:str, liquid_document: dict):
        return cls(db_name, liquid_document)

    @classmethod
    def from_object_id(cls, db_name:str, object_id: ObjectId):
        liquid_document = {'object_id': object_id}
        return cls(db_name, liquid_document)

    @property
    def density(self):
        #return in g/l which is equivalent to kg/m3
        return self.get_field('density')

    @property
    def pump_id(self):
        return self.get_field('pump_id')

    @property
    def volume(self):
        # return in l
        return self.get_field('volume')

    @volume.setter
    def volume(self, value):
        if value >= 0:
            self.update_field('volume', value)
        else:
            raise ValueError

    def __str__(self):
        return f'Liquid: {self.name}, Pump ID: {self.pump_id}, Expiry date: {self.expiry_date},\
                 Mass: {self.mass} g, Volume: {self.volume} L,\
                 Density: {self.density} g/L'


class Solid(Material):
    def __init__(self, db_name: str, solid_document: dict):
        if solid_document['unit'] == 'g':
            solid_document['mass'] = solid_document['amount_stored']
        elif solid_document['unit'] == 'mg':
            solid_document['mass'] = solid_document['amount_stored']/1000
        elif solid_document['unit'] == 'ug':
            solid_document['mass'] = solid_document['amount_stored']/1000000
        solid_document.pop('amount_stored')
        
        super().__init__(db_name, solid_document)

    @classmethod
    def from_dict(cls, db_name:str, solid_document: dict):
        return cls(db_name, solid_document)

    @classmethod
    def from_object_id(cls, db_name:str, object_id: ObjectId):
        solid_document = {'object_id': object_id}
        return cls(db_name, solid_document)
    
    @property
    def dispense_method(self):
        return self.get_field('dispense_method')

    @property
    def cartridge_id(self):
        return self.get_field('cartridge_id')

    def __str__(self):
        return f'Solid: {self.name}, Cartridge ID: {self.cartridge_id}, Expiry date: {self.expiry_date},\
                 Mass: {self.mass} g, Dispense method: {self.dispense_method}'
