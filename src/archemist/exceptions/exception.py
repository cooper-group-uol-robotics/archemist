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
