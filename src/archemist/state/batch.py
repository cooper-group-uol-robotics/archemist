from enum import Enum
from datetime import datetime


class Batch:

    class Status(Enum):
        NEED_TRANSIT = 1
        IN_TRANSIT = 2
        PROCESSING = 3
        UNDEFINED = -1

    def __init__(self, name: str, id: int):
        self.name = name
        self.id = id
        self.status = self.Status.UNDEFINED
        # self.location Location.UNDEFINED Location can be read from yaml
        #self.operationTimeStamp['Added to workflow'] = datetime.now()
