from archemist.state.state import State
from archemist.state.station import StationState
from archemist.persistence.persistenceManager import Parser

parser = Parser()
state = State()
state.initializeState(False)
ika = state.stations[0]
sm = parser.create_process_sm(ika.__class__.__name__)
sm.set_station(ika)
