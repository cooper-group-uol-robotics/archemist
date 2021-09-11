from archemist.processing.prcessor import WorkflowManager
from archemist.util.location import Location
from time import sleep
wm = WorkflowManager()
wm.initializeWorkflow()
wm.createBatch(1,1,1,Location(8,7,'input_frame'))
# set panda and kuka location
wm.process()