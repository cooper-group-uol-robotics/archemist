class Error(Exception):
    pass


class StationAssignedRackError(Error):
    def __init__(self, station_name):
        self.message = f'{station_name} station already has an assigned rack!!!'

class StationUnAssignedRackError(Error):
    def __init__(self, station_name):
        self.message = f'{station_name} station has no assigned rack!!!'
