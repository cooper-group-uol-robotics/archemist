from archemist.application.archemist_server import ArchemistServer
from pathlib import Path
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='script to run archemist server')
    parser.add_argument('--conf', dest='config_dir', action='store', type=str,
                    help='path to the config directory')
    parser.add_argument('--ex', dest='existing_db', action='store_true',
                    help='run server with already existing database')
    args = parser.parse_args()
    if args.config_dir is None:
        config_dir = Path.cwd()
    else:
        config_dir = Path(args.config_dir)
    existing_db = args.existing_db
    server = ArchemistServer(config_dir,existing_db)
    server.run()