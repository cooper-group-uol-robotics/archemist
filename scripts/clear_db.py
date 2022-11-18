#!/usr/bin/env python3

from archemist.core.persistence.db_handler import DatabaseHandler
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='script to clean up the stored db')
    parser.add_argument('-db', dest='db_name', action='store',
                    help='datebase name to be deleted', required=True)
    args = parser.parse_args()
    mongodb_host = 'mongodb://localhost:27017'
    db_handler = DatabaseHandler(mongodb_host,args.db_name)