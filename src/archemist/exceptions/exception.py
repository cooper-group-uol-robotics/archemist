class Error(Exception):
    pass


class StationAssignedRackError(Error):
    def __init__(self, station_name):
        self.message = f'{station_name} station already has an assigned rack!!!'

class StationUnAssignedRackError(Error):
    def __init__(self, station_name):
        self.message = f'{station_name} station has no assigned rack!!!'

class RobotAssignedRackError(Error):
    def __init__(self, robot_name):
        self.message = f'{robot_name} Robot already has an assigned rack!!!'

class RobotUnAssignedRackError(Error):
    def __init__(self, robot_name):
        self.message = f'{robot_name} Robot has no assigned rack!!!'

class StationNoOutcomeError(Error):
    def __init__(self, station_name):
        self.message = f'{station_name} station hasn not completed and can not provide an outcome result'

class InvalidLiquidError(Error):
    def __init__(self, station_name):
        self.message = f'{station_name} station does not contain the specified liquid'

class UsingConsumedCatridgeError(Error):
    def __init__(self, catridge_id):
        self.message = f'{catridge_id} is being used while it is consumed catridge'

class QuantosCatridgeLoadedError(Error):
    def __init__(self):
        self.message = f'Quantos station already has a loaded catridge!!!'

class QuantosRackLoadedError(Error):
    def __init__(self):
        self.message = f'Quantos station already has a loaded rack!!!'

class QuantosCatridgeUnLoadError(Error):
    def __init__(self):
        self.message = f'Quantos station does not have a loaded catridge!!!'

class DatabaseNotPopulatedError(Error):
    def __init__(self):
        self.message = f'The provided database is not existing or does not have any documents!!!'
