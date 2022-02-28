#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
##########################################################################
#
#  mongodb_skillsim_lookup.py: provide data for the lookup estimator
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
Extract skill durations from the skiller messages obtained via mongodb
blackboard logger or sample durations from mixed gaussian distributions.
"""

import argparse
import pymongo
import re
import textwrap
import sys
import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as ss

# Accourding to SkillStatusEnum
S_INACTIVE = 0
S_RUNNING = 2
S_FINAL = 1
S_FAILED = 3


def time_diff_in_sec(end, start):
    """ Computee the time diff in seconds from ms precision times. """
    return int(max((end - start) / 1000, 0))


def split_skill_string(skill):
    """ Separate skill name and arguments from a fawkes skill string. """
    skill = skill.strip().replace(" ", "").replace("\n", "")
    skill_regex = "([\w_-]+)\{(.*)\}"
    try_match = re.match(skill_regex, skill)
    if try_match:
        name, args_str = try_match.group(1, 2)
        arg_regex = '([\w_-]+)=\\"([\w_-]+)\\"(?:,)?'
        # split along the regex, filtering empty matches
        arg_list = list(filter(None, re.split(arg_regex, args_str)))
        # convert list to dictionary with even odd matching
        # see https://stackoverflow.com/a/23286311
        args = dict(zip(arg_list[::2], arg_list[1::2]))
        return name, args
    else:
        raise Exception("invalid skill format", skill)


def drop_collection(mongodb_uri, database, collection):
    """ Drop a collection from a mongodb instance. """
    curr_client = pymongo.MongoClient(mongodb_uri)
    curr_client[database][collection].drop()


class GaussSampler:
    """ Sampler to obtain points from a meixture of gaussians. """

    def __init__(self, quantity, dist_weights, gauss_params, upper_bound, lower_bound):
        """ Sample points from a bounded and weighted mixture of gaussians. """
        self.dist_weights = dist_weights
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        if len(self.dist_weights) != len(gauss_params):
            print(
                "Number of distribution weights do not match number of distributions!"
            )
            diff = len(gauss_params) - len(dist_weights)
            if diff < 0:
                print("Ignoring trailing distribution weights")
                self.dist_weights = self.dist_weights[: len(dist_weights) + diff]
            else:
                print("Assuming default weights of 1")
                self.dist_weights.extend([1] * diff)
        # normalize weights
        self.dist_weights = np.array(
            [float(i) / sum(self.dist_weights) for i in self.dist_weights]
        )
        # create samples
        self.samples = []
        self.gauss_params = gauss_params
        sample_size = quantity
        self.sample_min, self.sample_max = [float("inf"), -float("inf")]
        while True:
            # determine the gaussian to sample from for each sample
            mixture_idx = np.random.choice(
                len(self.dist_weights),
                size=sample_size,
                replace=True,
                p=self.dist_weights,
            )
            # create the samples from the respective gaussian
            temp = np.fromiter(
                (ss.norm.rvs(*(gauss_params[i])) for i in mixture_idx), dtype=np.float64
            )
            # remember mixed sampled extremas for plotting
            self.sample_min = min(self.sample_min, temp.min())
            self.sample_max = max(self.sample_max, temp.max())
            # add those samples that are within the bounds
            self.samples = np.concatenate(
                [
                    self.samples,
                    np.fromiter(
                        [x for x in temp if x <= upper_bound and x >= lower_bound],
                        dtype=np.float64,
                    ),
                ]
            )
            sample_size = quantity - len(self.samples)
            if sample_size == 0:
                break

    def display(self, bin_size):
        """ Show the sampled points using pyplot. """
        xs = np.linspace(self.sample_min, self.sample_max, 2000)
        ys = np.zeros_like(xs)
        for (l, s), w in zip(self.gauss_params, self.dist_weights):
            ys += ss.norm.pdf(xs, loc=l, scale=s) * w
        plt.plot(xs, ys, color="blue")
        plt.hist(self.samples, density=True, bins=bin_size, color="palegreen")
        plt.xlabel("duration")
        plt.ylabel("density")
        _, _, ymin, ymax = plt.axis()
        if self.lower_bound > 0:
            plt.vlines([self.lower_bound], ymin, ymax, color="crimson")
        if self.upper_bound < float("inf"):
            plt.vlines([self.upper_bound], ymin, ymax, color="crimson")
        plt.show()


class MongoInterface:
    """ Interface to a mongodb instance. """

    def __init__(self, dst_mongodb_uri, dst_database, dst_collection, dry_run):
        """ Set up a connection to a mongodb collection. """
        self.dry_run = dry_run
        if not self.dry_run:
            self.client = pymongo.MongoClient(dst_mongodb_uri)
            self.dst_mongodb_uri = dst_mongodb_uri
            self.lookup_col = self.client[dst_database][dst_collection]

    def upload(self, durations, skill_name, skill_args, failures):
        """ Upload skill exec time entries, randomly sample from skill arg
            choices and possible failures. """
        print(failures)
        for dur in durations:
            outcome = 1
            error = ""
            for fail_rate, error_msg in failures:
                if np.random.random_sample() < float(fail_rate):
                    outcome = 3
                    error = error_msg
                    break
            doc = {"outcome": outcome, "error": error, "name": skill_name, "duration": dur}
            args_dict = dict()
            if skill_args != None:
                for arg in skill_args:
                    if len(arg) == 1:
                        args_dict[arg[0]] = ".*"
                    else:
                        args_dict[arg[0]] = np.random.choice(arg[1:])
            doc["args"] = args_dict
            print(doc)
            if not self.dry_run:
                self.lookup_col.insert_one(doc)

    def transform(
        self, src_mongodb_uri, src_database, src_collection, lower_bound, upper_bound
    ):
        """ Transform blackboard log data from the Skiller to exec time entries. """
        if src_mongodb_uri != self.dst_mongodb_uri:
            self.clone_collection(src_mongodb_uri, src_database, src_collection)
        col = self.client[src_database][src_collection]
        for skill_start in col.find({"status": S_RUNNING}).sort("timestamp", 1):
            for skill_end in (
                col.find(
                    {
                        "skill_string": skill_start["skill_string"],
                        "thread": skill_start["thread"],
                        "timestamp": {"$gt": skill_start["timestamp"]},
                    }
                )
                .sort("timestamp", 1)
                .limit(1)
            ):
                if skill_end["status"] == S_FINAL or skill_end["status"] == S_FAILED:
                    name, args = split_skill_string(skill_start["skill_string"])
                    lookup_entry = {
                        "_id": {
                            "thread": skill_start["thread"],
                            "start_time": skill_start["timestamp"],
                            "end_time": skill_end["timestamp"],
                        },
                        "outcome": skill_end["status"],
                        "error": skill_end["error"],
                        "name": name,
                        "args": args,
                        "duration": time_diff_in_sec(
                            skill_end["timestamp"], skill_start["timestamp"]
                        ),
                    }
                    if (
                        lookup_entry["duration"] > upper_bound
                        or lookup_entry["duration"] < lower_bound
                    ):
                        print(
                            "duration out of bounds, omitting: {} seconds\n{}\n{}".format(
                                lookup_entry["duration"], skill_start, skill_end
                            )
                        )
                    else:
                        if not self.lookup_col.find_one(lookup_entry):
                            if not self.dry_run:
                                self.lookup_col.insert_one(lookup_entry)
                            print("Adding: {}".format(lookup_entry))
                        else:
                            print("Entry already present, omitting")
        if src_mongodb_uri != self.dst_mongodb_uri:
            self.client.drop_database(src_database)

    def clone_collection(self, src_mongodb_uri, src_database, src_collection):
        """ Clone a mongodb collection to the client. """
        # drop "mongodb://" suffix from uri
        src_conn = src_mongodb_uri[10:]
        if src_conn[-1] == "/":
            src_conn = src_conn[:-1]
        self.client.admin.command(
            {"cloneCollection": src_database + "." + src_collection, "from": src_conn}
        )


def main():
    """ Obtain data for the lookup execution time estimator. """
    header = """
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
"""
    parser = argparse.ArgumentParser(
        description=textwrap.dedent(header),
        formatter_class=argparse.RawTextHelpFormatter,
    )
    common = argparse.ArgumentParser(add_help=False)
    group = common.add_argument_group("Mongodb options")
    group.add_argument(
        "--mongodb-uri",
        type=str,
        help="The MongoDB URI of the execution time estimator lookup database (default: %(default)s)",
        default="mongodb://localhost:27017/",
    )
    group.add_argument(
        "--db",
        type=str,
        help=textwrap.dedent("""name of the lookup database (default: %(default)s)"""),
        default="skills",
    )
    group.add_argument(
        "--dry-run",
        "-d",
        action="store_true",
        help="only create samples without uploading them to mongodb",
    )
    group.add_argument(
        "--collection",
        "-c",
        type=str,
        help="name of the lookup collection (default: %(default)s)",
        default="exec_times",
    )
    group.add_argument(
        "--drop-collection-first",
        "-dc",
        action="store_true",
        help="clear all old data from the collection",
    )
    subparsers = parser.add_subparsers(
        help="Source of the execution time data", dest="subparser"
    )
    bb_parser = subparsers.add_parser(
        "bblog",
        parents=[common],
        description=textwrap.dedent(
            header
            + """\
#                                                                             #
# Selected option 1                                                           #
#                                                                             #
###############################################################################
"""
        ),
        formatter_class=argparse.RawTextHelpFormatter,
    )
    bb_parser.set_defaults()
    random_parser = subparsers.add_parser(
        "generate",
        parents=[common],
        description=textwrap.dedent(
            header
            + """\
#                                                                             #
# Selected option 2                                                           #
#                                                                             #
###############################################################################
"""
        ),
        formatter_class=argparse.RawTextHelpFormatter,
    )
    random_parser.set_defaults()
    bb_sanity = bb_parser.add_argument_group("Sanity checks to avoid faulty entries")
    bb_sanity.add_argument(
        "--lower-bound",
        "-l",
        type=float,
        default=0,
        help="ignore entries with duration smaller than this",
    )
    bb_sanity.add_argument(
        "--upper-bound",
        "-u",
        type=float,
        default=float("inf"),
        help="ignore entries with duration smaller than this",
    )
    bb_log = bb_parser.add_argument_group("Blackboard log information")
    bb_log.add_argument(
        "--src-uri",
        type=str,
        help="The MongoDB URI of the blackboard log connection (default: %(default)s)",
        default="mongodb://localhost:27017/",
    )
    bb_log.add_argument(
        "--src-db",
        type=str,
        help="The name of the blackboard log database (default: %(default)s)",
        default="fflog",
    )
    bb_log.add_argument(
        "--src-col",
        type=str,
        help="The name of the blackboard log collection (default: %(default)s)",
        default="SkillerInterface.Skiller",
    )
    bb_log.add_argument(
        "--drop-src-col",
        type=bool,
        help="Delete the skiller blackboard log collection afterwards",
        default=False,
    )

    skill = random_parser.add_argument_group("Skill information")
    skill.add_argument(
        "--quantity",
        "-n",
        type=int,
        help="number of entries to generate",
        required=True,
    )
    skill.add_argument(
        "--inject-failure",
        type=str,
        help="Percentage of failed skills and reason",
        nargs=2,
        action="append",
    )
    skill.add_argument(
        "--skill-name",
        "-s",
        type=str,
        help="skill name to generate entries for",
        required=True,
    )
    skill.add_argument(
        "--skill-args",
        "-a",
        type=str,
        nargs="+",
        action="append",
        help=textwrap.dedent(
            """skill arguments. usage -a <arg-name> <val1> <val2> ...
                              where val<i> are the possible values of the argument that will be chosen from at random
                              * (placeholder value) if no values are given
                           """
        ),
    )

    gauss = random_parser.add_argument_group("Mixed gaussian distribution")
    gauss.add_argument(
        "--gauss-params",
        "-g",
        type=float,
        help="mean and standard deviation (in that order) of a gaussian, repeat this option to add more gaussians",
        nargs=2,
        required=True,
        action="append",
    )
    gauss.add_argument(
        "--dist-weights",
        "-w",
        type=float,
        default=[],
        help="Weight of each gauss distribution (default 1)",
        nargs="+",
    )
    gauss.add_argument(
        "--lower-bound",
        "-l",
        type=float,
        default=0,
        help="clip distribution to a lower bound",
    )
    gauss.add_argument(
        "--upper-bound",
        "-u",
        type=float,
        default=float("inf"),
        help="clip distribution to an upper bound",
    )

    visual = random_parser.add_argument_group("Visualization options")
    visual.add_argument(
        "--bin-size",
        "-b",
        type=int,
        help="number of bins to display sampled durations (default: %(default)s)",
        default=50,
    )
    visual.add_argument(
        "--non-interactive",
        "-y",
        action="store_true",
        help="skip drawing the sample range",
    )
    parser.epilog = (
        "--- Arguments common to all sub-parsers ---"
        + common.format_help().replace(common.format_usage(), "")
    )
    random_parser.epilog = """
example call: ./mongodb_skillsim_lookup.py generate -d -n \
1000 -g 10 2 -g 20 3 -w 1 5 -s test -a arg1 value1 value2 -a arg2
  """
    args = parser.parse_args(args=None if sys.argv[1:] else ["--help"])
    # validate inputs
    if args == None:
        parser.exit(1)

    mongoIf = MongoInterface(args.mongodb_uri, args.db, args.collection, args.dry_run)
    if args.drop_collection_first and not args.dry_run:
        print("Drop collection before uploading...")
        drop_collection(args.mongodb_uri, args.db, args.collection)
    if args.subparser == "bblog":
        mongoIf.transform(
            args.src_uri, args.src_db, args.src_col, args.lower_bound, args.upper_bound
        )
        if args.drop_src_col:
            drop_collection(args.src_mongodb_uri, args.src_db, args.src_col)
    elif args.subparser == "generate":
        sampler = GaussSampler(
            args.quantity,
            args.dist_weights,
            args.gauss_params,
            args.upper_bound,
            args.lower_bound,
        )
        if not args.non_interactive:
            sampler.display(args.bin_size)
        mongoIf.upload(sampler.samples, args.skill_name, args.skill_args,args.inject_failure)
    else:
        print("unrecognized mode")


if __name__ == "__main__":
    main()
