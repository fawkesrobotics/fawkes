#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
##########################################################################
#
#  mongodb_bb_connector.py transform mongo skiller logs to lookup entries
#
#  Copyright © 2020 Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
#
##########################################################################
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Library General Public License for more details.
#
#  Read the full text in the LICENSE.GPL file in the doc directory.


"""
Extract skill durations from the skiller messages obtained via mongodb blackboard logger.
"""

import argparse
import pymongo
import re

# Accourding to SkillStatusEnum
S_INACTIVE = 0
S_RUNNING = 2
S_FINAL = 1
S_FAILED = 3

def time_diff_in_sec(end, start):
    return int(max((end - start)/1000, 0))

def split_skill_string(skill):
    skill = skill.strip().replace(" ","").replace("\n","")
    skill_regex = "([\w_-]+)\{(.*)\}"
    try_match = re.match(skill_regex, skill)
    if try_match:
        name, args_str = try_match.group(1,2)
        arg_regex = '([\w_-]+)=\\"([\w_-]+)\\"(?:,)?'
        # split along the regex, filtering empty matches
        arg_list = list(filter(None,re.split(arg_regex, args_str)))
        # convert list to dictionary with even odd matching
        # see https://stackoverflow.com/a/23286311
        args = dict(zip(arg_list[::2], arg_list[1::2]))
        return name, args
    else:
        raise Exception("invalid skill format", skill)

def drop_collection(mongodb_uri, database, collection):
  curr_client = pymongo.MongoClient(mongodb_uri)
  curr_client[database][collection].drop()

class MongoInterface:

  def __init__(
          self,
          dst_mongodb_uri,
          dst_database,
          dst_collection,
          dry_run):
    self.client = pymongo.MongoClient(dst_mongodb_uri)
    self.dst_mongodb_uri = dst_mongodb_uri
    self.lookup_col = self.client[dst_database][dst_collection]
    self.dry_run = dry_run

  def transform(self, src_mongodb_uri, src_database, src_collection, lower_bound, upper_bound):
    if src_mongodb_uri != self.dst_mongodb_uri:
      self.clone_collection(src_mongodb_uri, src_database, src_collection)
    col = self.client[src_database][src_collection]
    for skill_start in col.find({"status": S_RUNNING}).sort("timestamp", 1):
        for skill_end in col.find({"skill_string": skill_start["skill_string"],
                               "thread": skill_start["thread"],
                               "timestamp": {"$gt": skill_start["timestamp"]}
                              }).sort("timestamp", 1).limit(1):
          if skill_end["status"] == S_FINAL or skill_end["status"] == S_FAILED:
            name, args = split_skill_string(skill_start["skill_string"])
            lookup_entry = {"_id": {"thread": skill_start["thread"],
                                    "start_time": skill_start["timestamp"],
                                    "end_time": skill_end["timestamp"]},
                            "outcome": skill_end["status"],
                            "error": skill_end["error"],
                            "name": name,
                            "args": args,
                            "duration": time_diff_in_sec(skill_end["timestamp"],skill_start["timestamp"])}
            if lookup_entry["duration"] > upper_bound or lookup_entry["duration"] < lower_bound:
                print("duration out of bounds, omitting: {} seconds\n{}\n{}".format(lookup_entry["duration"], skill_start, skill_end))
            else:
              if not self.lookup_col.find_one(lookup_entry):
                if(not self.dry_run):
                  self.lookup_col.insert_one(lookup_entry)
                print("Adding: {}".format(lookup_entry))
              else:
                print("Entry already present, omitting")
    if src_mongodb_uri != self.dst_mongodb_uri:
      self.client.drop_database(src_database)

  def clone_collection(self, src_mongodb_uri, src_database,src_collection):
    # drop "mongodb://" suffix from uri
    src_conn = src_mongodb_uri[10:]
    if src_conn[-1] == "/":
        src_conn = src_conn[:-1]
    self.client.admin.command({'cloneCollection': src_database+"."+src_collection, 'from': src_conn})


def main():
  header = '''
###############################################################################
#                                                                             #
#   Obtain data for the lookup execution time estimator                       #
#                                                                             #
# --------------------------------------------------------------------------- #
#                                                                             #
#   Import execution times to mongodb from                                    #
#   1. mongodb_log via recorded blackboard skiller calls                      #
#   2. samples of a mixed gaussian distribution                               #
#                                                                             #
###############################################################################
'''
  parser = argparse.ArgumentParser(description=textwrap.dedent(header),
                                   formatter_class=argparse.RawTextHelpFormatter)
  common = argparse.ArgumentParser(add_help=False)
  group = common.add_argument_group("Mongodb options")
  group.add_argument(
      '--mongodb-uri',
      type=str,
      help='The MongoDB URI of the execution time estimator lookup database',
      default='mongodb://localhost:27017/')
  group.add_argument(
      '--db',
      type=str,
      help=textwrap.dedent('''name of the lookup database (default: %(default)s)'''),
      default='skills')
  group.add_argument(
      '--dry-run', '-d',
      action='store_true',
      help='only create samples without uploading them to mongodb')
  group.add_argument(
      '--collection', '-c',
      type=str,
      help='name of the lookup collection',
      default='exec_times')
  group.add_argument(
      '--drop-collection-first', '-dc',
      action='store_true',
      help='clear all old data from the collection')
  subparsers = parser.add_subparsers(help='Source of the execution time data',dest='subparser')
  bb_parser = subparsers.add_parser("bblog",parents=[common],
                                    description=textwrap.dedent(header + '''\
#                                                                             #
# Selected option 1                                                           #
#                                                                             #
###############################################################################
'''),formatter_class=argparse.RawTextHelpFormatter)
  bb_parser.set_defaults()
  bb_sanity = bb_parser.add_argument_group("Sanity checks to avoid faulty entries")
  bb_sanity.add_argument(
      '--lower-bound', '-l',
      type=float,
      default=0,
      help='ignore entries with duration smaller than this')
  bb_sanity.add_argument(
      '--upper-bound', '-u',
      type=float,
      default=float('inf'),
      help='ignore entries with duration smaller than this')
  bb_log = bb_parser.add_argument_group("Blackboard log information")
  bb_log.add_argument(
      '--src-uri',
      type=str,
      help='The MongoDB URI of the blackboard log connection',
      default='mongodb://localhost:27017/')
  bb_log.add_argument(
      '--src-db',
      type=str,
      help='The name of the blackboard log database',
      default='fflog')
  bb_log.add_argument(
      '--src-col',
      type=str,
      help='The name of the blackboard log collection',
      default='SkillerInterface.Skiller')
  bb_log.add_argument(
      '--drop-src-col',
      type=bool,
      help='Delete the skiller blackboard log collection afterwards',
      default=False)

  args = parser.parse_args(args=None if sys.argv[1:] else ['--help'])
  # validate inputs
  if args==None:
      parser.exit(1)

  mongoIf = MongoInterface(args.mongodb_uri,args.db,args.collection,args.dry_run)
  if(args.drop_collection_first and not args.dry_run):
      print("Drop collection before uploading...")
      drop_collection(args.mongodb_uri,args.db,args.collection)
  if args.subparser == 'bblog':
    mongoIf.transform(args.src_uri,args.src_db,args.src_col,
                      args.lower_bound,args.upper_bound)
    if args.drop_src_col:
        drop_collection(args.src_mongodb_uri, args.src_db, args.src_col)
  else:
    print("unrecognized mode")


if __name__ == '__main__':
  main()
