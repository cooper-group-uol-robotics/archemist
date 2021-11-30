from archemist.state.state import State
from archemist.state.station import StationState
from archemist.persistence.persistenceManager import Parser

parser = Parser()
state = State()
state.initializeState(False)
fish = state.stations[1]
sm = parser.create_process_sm(fish.__class__.__name__)
sm.set_station(fish)
#state.updateFromDB()
