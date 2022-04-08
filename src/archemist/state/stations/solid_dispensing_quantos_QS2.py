from archemist.state.station import Station, StationOpDescriptor, StationOutputDescriptor
from archemist.state.material import Solid
from archemist.exceptions.exception import UsingConsumedCatridgeError, QuantosCatridgeLoadedError
from bson.objectid import ObjectId


class QuantosCatridge():
    def __init__(self, catridge_dict: dict):
        self._solid_name = catridge_dict['solid_name']
        self._solid_obj_id = catridge_dict['solid_obj_id']
        self._db_name = catridge_dict['db_name']
        self._remaining_dosages = catridge_dict['remaining_dosages']
        self._blocked = catridge_dict['blocked']
        self._consumed = catridge_dict['consumed']
        self._hotel_id = catridge_dict['hotel_id']

    @classmethod
    def from_dict(cls, catridge_dict: dict):
        return cls(catridge_dict)

    @classmethod
    def from_config_dict(cls, catridge_dict: dict):
        catridge_dict.update({'blocked': False, 'consumed': False})
        return cls(catridge_dict)


    @property
    def solid_name(self):
        return self._solid_name

    @property
    def solid(self):
        return Solid.from_object_id(self._db_name, self._solid_obj_id)

    @property
    def hotel_id(self):
        return self._hotel_id

    @property
    def consumed(self):
        return self._consumed

    @property
    def blocked(self):
        return self._blocked

    @property
    def remaining_dosages(self):
        return self._remaining_dosages

    def to_dict(self):
        return{
            'solid_name': self._solid_name,
            'solid_obj_id': self._solid_obj_id,
            'db_name': self._db_name,
            'remaining_dosages': self._remaining_dosages,
            'blocked': self._blocked,
            'consumed': self._consumed,
            'hotel_id': self._hotel_id
        }

    # def dispense(self, dispensed_solid: Solid):
    #     if (not self._consumed):
    #         self._solid.mass = self._solid.mass - dispensed_solid.mass
    #         self._remaining_dosages -= 1
    #         if (self._remaining_dosages <= 0):
    #             self._consumed = True
    #     else:
    #         raise UsingConsumedCatridgeError(self._hotel_id)





class QuantosSolidDispenserQS2(Station):
    def __init__(self, db_name: str, station_dict: dict, liquids: list, solids: list):
        if len(station_dict) > 1:
            parameters = station_dict.pop('parameters')
            station_dict['carousel_pos'] = -1
            station_dict['catridges'] = dict()
            for cat in parameters['catridges']:
                for solid in solids:
                    if (solid.cartridge_id == cat['hotel_id']):
                        cat['solid_obj_id'] = solid.object_id
                        cat['db_name'] = solid.db_name
                        catridge = QuantosCatridge.from_config_dict(cat)
                        station_dict['catridges'].update({cat['hotel_id']: catridge.to_dict()})
            station_dict['current_loaded_catridge'] = None
            station_dict['doors_open'] = False

        super().__init__(db_name,station_dict)

    @classmethod
    def from_dict(cls, db_name: str, station_dict: dict, liquids: list, solids: list):
        return cls(db_name, station_dict, liquids, solids)

    @classmethod
    def from_object_id(cls, db_name: str, object_id: ObjectId):
        station_dict = {'object_id':object_id}
        return cls(db_name, station_dict, None, None)

    @property
    def carousel_pos(self):
        return self.get_field('carousel_pos')

    @carousel_pos.setter
    def carousel_pos(self, value: int):
        if (value <= 20 and value >= 1):
            self.update_field('carousel_pos', value)
        else:
            raise ValueError

    @property
    def current_catridge(self):
        cat_id = self.get_field('current_loaded_catridge')
        if cat_id is not None:
            catridge_dict = self.get_nested_field(f'catridges.{cat_id}')
            return QuantosCatridge.from_dict(catridge_dict)

    @property
    def doors_open(self):
        return self.get_field('doors_open')

    @doors_open.setter
    def doors_open(self, value):
        if isinstance(value, bool):
            self.update_field('doors_open', value)
        else:
            raise ValueError

    def get_catridge_id(self, solid_name: str):
        catridges = self.get_field('catridges')
        for cat_id, cat_dict in catridges.items():
            if cat_dict['solid_name'] == solid_name:
                return cat_id

    def load_catridge(self, catridge_id: str):
        if (self.current_catridge is None):
            self.update_field('current_loaded_catridge', catridge_id)
        else:
            raise QuantosCatridgeLoadedError()

    def unload_current_catridge(self):
        if (self.current_catridge is not None):
            self.update_field('current_loaded_catridge', None)
        else:
            print('unloading catridge when no catridge is loaded!!!')

    def dispense(self, catridge_id: str, dispense_amount: float):
        current_catridge = self.current_catridge
        if not current_catridge.consumed and not current_catridge.blocked:
            current_solid = self.current_catridge.solid
            current_solid.mass = current_solid.mass - dispense_amount
            self.decrement_field(f'catridges.{catridge_id}.remaining_dosages')
            if self.get_nested_field(f'catridges.{catridge_id}.remaining_dosages') == 0:
                self._log_station(f'the catridge {catridge_id} does not have any dosages left anymore. Please replace it.')
                self.update_field(f'catridges.{catridge_id}.consumed', True)
        else:
            self._log_station(f'Current catridge {catridge_id} is either consumed or blocked. Cannot Dispense solid!!!')


''' ==== Station Operation Descriptors ==== '''

class QuantosDispenseOpDescriptor(StationOpDescriptor):
    def __init__(self, properties: dict, output: StationOutputDescriptor):
        super().__init__(stationName=QuantosSolidDispenserQS2.__class__.__name__, output=output)
        self._solid_name = properties['solid']
        self._dispense_mass = properties['mass']

    @property
    def solid_name(self):
        return self._solid_name

    @property
    def dispense_mass(self):
        return self._dispense_mass

# TODO you can have another op for loading catridge and doing stuff


''' ==== Station Output Descriptors ==== '''

class QuantosOutputDescriptor(StationOutputDescriptor):
    def __init__(self):
        super().__init__()
