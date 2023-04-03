from archemist.core.processing.prcessor import WorkflowManager
from archemist.core.util.location import Location

wm = WorkflowManager()
wm.initializeWorkflow()
wm.createBatch(1, 1, 1, Location(7, 7, "quantos_rack"))
# set panda and kuka location
wm.process()
