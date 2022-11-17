from archemist.application.archemist_server import ArchemistServer
from pathlib import Path
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='script to run archemist server')
    parser.add_argument('--path', dest='workflow_path', action='store', type=str,
                    help='path to the workflow directory', required=True)
    parser.add_argument('--exists', dest='existing_db', action='store_true',
                    help='run server with already existing database')
    args = parser.parse_args()
    
    workflow_dir = Path(args.workflow_path)
    if not workflow_dir.exists():
        print('The given workflow directory does not exist.')
        print( 'Please check your path or create a new directory using create_workflow_dir script')
        exit()
    existing_db = args.existing_db
    server = ArchemistServer(workflow_dir,existing_db)
    server.run()