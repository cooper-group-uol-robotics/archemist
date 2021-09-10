from datetime import date


class Material:
    def __init__(self, name: str, expiry_date: date, mass: float):
        self._name = name
        self._expiry_date = expiry_date
        self._mass = mass

    @property
    def name(self):
        return self._name

    @property
    def expiry_date(self):
        return self._expiry_date

    @property
    def mass(self):
        return self._mass

    @mass.setter
    def mass(self, value):
        if value >= 0:
            self._mass = value
        else:
            raise ValueError


class Liquid(Material):
    def __init__(self, name: str, pump_id: str, expiry_date: date, mass: float,
                 density: float, volume: float):
        super().__init__(name, expiry_date, mass)
        self._density = density
        self._volume = volume
        self._pump_id = pump_id

    @property
    def density(self):
        return self._density

    @property
    def pump_id(self):
        return self._pump_id


    @density.setter
    def density(self, value):
        if value >= 0:
            self._density = value
        else:
            raise ValueError

    @property
    def volume(self):
        return self._volume

    @volume.setter
    def volume(self, value):
        if value >= 0:
            self._volume = value
        else:
            raise ValueError

    def __str__(self):
        return f'Liquid: {self._name}, Pump ID: {self._pump_id}, Expiry date: {self._expiry_date},\
                 Mass: {self._mass} g, Volume: {self._volume} L,\
                 Density: {self._density} g/L'


class Solid(Material):
    def __init__(self, name: str, cartridge_id: str, expiry_date: date, mass: float,
                 dispense_method: str):
        super().__init__(name, expiry_date, mass)
        self._dispense_method = dispense_method
        self._cartridge_id = cartridge_id
    @property
    def dispense_method(self):
        return self._dispense_method

    @dispense_method.setter
    def dispense_method(self, value):
        self._dispense_method = value

    @property
    def cartridge_id(self):
        return self._cartridge_id

    def __str__(self):
        return f'Solid: {self._name}, Cartridge ID: {self._cartridge_id}, Expiry date: {self._expiry_date},\
                 Mass: {self._mass} g, Dispense method: {self._dispense_method}'
