from archemist.processing.prcessor import WorkflowManager
from archemist.util.location import Location

wm = WorkflowManager()
wm.initializeWorkflow()
wm.createBatch(1,1,Location(1,7,'/ikaStation/RackHolderFrame'))
