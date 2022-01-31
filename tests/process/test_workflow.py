from archemist.processing.prcessor import WorkflowManager
from archemist.persistence.persistenceManager import PersistenceManager
from archemist.util.location import Location
import os   

current_dir = os.path.dirname(os.path.realpath(__file__))
config_file_path = os.path.abspath(os.path.join(current_dir, '../state/resources/testing_config_file.yaml'))
pm = PersistenceManager('test')
state = pm.construct_state_from_config_file(config_file_path)
wm = WorkflowManager(state)
