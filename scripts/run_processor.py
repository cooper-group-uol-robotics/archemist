from archemist.processing.prcessor import WorkflowProcessor

if __name__ == '__main__':
    try:
        wf_processor = WorkflowProcessor()
        wf_processor.process()
    except KeyboardInterrupt:
        print('Workflow processor terminating')