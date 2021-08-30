from datetime import date


class Material:
    def __init__(self, name: str, id: int, expiry_date: date,
                 mass: float):
        self._name = name
        self._id = id
        self._expiry_date = expiry_date
        self._mass = mass

    @property
    def name(self):
        return self._name

    @property
    def id(self):
        return self._id

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
    def __init__(self, name: str, id: int, expiry_date: date, mass: float,
                 density: float, volume: float):
        super().__init__(name, id, expiry_date, mass)
        self._density = density
        self._volume = volume

    @property
    def density(self):
        return self._density

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
        return f'Liquid: {self._name}, ID: {self._id}, Expiry date: {self._expiry_date},\
                 Mass: {self._mass} g, Volume: {self._volume} L,\
                 Density: {self._density} g/L'


class Solid(Material):
    def __init__(self, name: str, id: int, expiry_date: date, mass: float,
                 dispense_method: str):
        super().__init__(name, id, expiry_date, mass)
        self._dispense_method = dispense_method

    @property
    def dispense_method(self):
        return self._dispense_method

    @dispense_method.setter
    def dispense_method(self, value):
        self._dispense_method = value

    def __str__(self):
        return f'Solid: {self._name}, ID: {self._id}, Expiry date: {self._expiry_date},\
                 Mass: {self._mass} g, Dispense method: {self._dispense_method}'
