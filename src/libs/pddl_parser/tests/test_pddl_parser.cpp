/***************************************************************************
 *  test_skill_parser.cpp - Tests for ExecutionTimeEstimator::Skill
 *
 *  Created: 23-10-2020
 *  Copyright  2021  Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */
#include "../pddl_exception.h"
#include "../pddl_parser.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>

using namespace pddl_parser;

TEST(PddlParserTest, TypingTest)
{
	auto benchmarks = {"(define (domain action-arg-pred-missmatch)       \n\
	(:requirements :strips :typing)             \n\
	(:types                                     \n\
	 	obj-a - object                            \n\
	 	obj-b - object                            \n\
	 )                                          \n\
	(:predicates                                \n\
	 	(pred ?r - obj-a)                         \n\
	 )                                          \n\
	(:action test-action                        \n\
	 :parameters (?t - obj-b)                   \n\
	 :precondition (pred ?t)                    \n\
	 :effect (not (pred ?t))                    \n\
	 )                                          \n\
)",
	                   "(define (domain typing-disabled-but-type-defined) \n\
	(:requirements :strips)                     \n\
	(:types                                     \n\
	 	obj-a - object                            \n\
	 )                                          \n\
	(:predicates                                \n\
	 	(pred ?r)                                 \n\
	 )                                          \n\
	(:action test-action                        \n\
	 :parameters (?t)                           \n\
	 :precondition (pred ?t)                    \n\
	 :effect (not (pred ?t))                    \n\
	 )                                          \n\
)",
	                   "(define (domain typing-disabled-but-type-constant) \n\
	(:requirements :strips)                     \n\
	(:constants                                 \n\
	 	TEST_CONST - object                       \n\
	 )                                          \n\
	(:predicates                                \n\
	 	(pred ?r)                                 \n\
	 )                                          \n\
	(:action test-action                        \n\
	 :parameters (?t)                           \n\
	 :precondition (pred ?t)                    \n\
	 :effect (not (pred ?t))                    \n\
	 )                                          \n\
)",
	                   "(define (domain typing-disabled-but-type-param) \n\
	(:requirements :strips)                     \n\
	(:predicates                                \n\
	 	(pred ?r)                                 \n\
	 )                                          \n\
	(:action test-action                        \n\
	 :parameters (?t - object)                  \n\
	 :precondition (pred ?t)                    \n\
	 :effect (not (pred ?t))                    \n\
	 )                                          \n\
)",
	                   "(define (domain action-unknown-type-in-pred)     \n\
	(:requirements :strips :typing)             \n\
	(:types                                     \n\
	 	obj-a - object                            \n\
	 )                                          \n\
	(:predicates                                \n\
	 	(pred ?r - obj-b)                         \n\
	 )                                          \n\
	(:action test-action                        \n\
	 :parameters (?t - obj-a)                   \n\
	 :precondition (pred ?t)                    \n\
	 :effect (not (pred ?t))                    \n\
	 )                                          \n\
)",
	                   "(define (domain action-unknown-type-in-param)    \n\
	(:requirements :strips :typing)             \n\
	(:types                                     \n\
	 	obj-a - object                            \n\
	 )                                          \n\
	(:predicates                                \n\
	 	(pred ?r - obj-a)                         \n\
	 )                                          \n\
	(:action test-action                        \n\
	 :parameters (?t - obj-b)                   \n\
	 :precondition (pred ?t)                    \n\
	 :effect (not (pred ?t))                    \n\
	 )                                          \n\
)",
	                   "(define (domain action-unknown-type-in-constant) \n\
	(:requirements :strips :typing)             \n\
	(:types                                     \n\
	 	obj-a - object                            \n\
	 )                                          \n\
	(:constants                                 \n\
	 	TEST_CONST - obj-b                        \n\
	 )                                          \n\
	(:predicates                                \n\
	 	(pred ?r - obj-a)                         \n\
	 )                                          \n\
	(:action test-action                        \n\
	 :parameters (?t - obj-a)                   \n\
	 :precondition (pred ?t)                    \n\
	 :effect (not (pred ?t))                    \n\
	 )                                          \n\
)",
	                   "(define (domain action-constant-missmatch)       \n\
	(:requirements :strips :typing)             \n\
	(:types                                     \n\
	 	obj-a - object                            \n\
	 	obj-b - object                            \n\
	 )                                          \n\
	(:constants                                 \n\
	 	TEST_CONST - obj-b                        \n\
	 )                                          \n\
	(:predicates                                \n\
	 	(pred ?r - obj-a)                         \n\
	 )                                          \n\
	(:action test-action                        \n\
	 :parameters (?t - obj-a)                   \n\
	 :precondition (pred TEST_CONST)            \n\
	 :effect (not (pred ?t))                    \n\
	 )                                          \n\
)"};
	for (const auto &s : benchmarks) {
		try {
			PddlParser p;
			p.parseDomain(s);
			FAIL() << s << "\n expected type error";
		} catch (PddlParserException &e) {
			if (e.error_type != PddlErrorType::TYPE_ERROR) {
				FAIL() << s << "\n failed with error " << e.what();
			} else {
			}
		}
	}
	SUCCEED() << " got expected type errors ";
}

TEST(PddlParserTest, MinimalDomain)
{
	try {
		PddlParser p;
		p.parseDomain("(define (domain test-domain)              \n\
	(:requirements)                  \n\
	(:predicates                     \n\
		(pred)                         \n\
	 )                               \n\
	(:action test-action             \n\
	 :parameters (?t)                \n\
	 :precondition (pred)            \n\
	 :effect (not (pred))            \n\
	 )                               \n\
)");
		SUCCEED() << "Minimal domain parsed ";
	} catch (PddlParserException const &e) {
		FAIL() << " Unexpected Exception: " << e.what();
	}
}

TEST(PddlParserTest, DurativeAction)
{
	try {
		PddlParser p;
		p.parseDomain("(define (domain test-durative-action)   \n\
	(:requirements :strips :durative-actions)    \n\
	(:predicates                                 \n\
		(pred ?r)                                  \n\
	 )                                           \n\
	(:durative-action test-action                \n\
	 :parameters (?t)                            \n\
	 :duration (= ?duration 5.5)                 \n\
	 :condition (and (at start (pred ?t))        \n\
										(over all (pred ?t)))      \n\
	 :effect (and (at end (not (pred ?t)))       \n\
	             (at start (pred ?t)))           \n\
	 )                                           \n\
)");
		SUCCEED() << " Durative domain parsed ";
	} catch (PddlParserException const &e) {
		FAIL() << " Unexpected Exception: " << e.what();
	}
}

TEST(PddlParserTest, Functionss)
{
	try {
		PddlParser p;
		p.parseDomain("(define (domain test-functions)   \n\
	(:requirements :strips :numeric-fluents)     \n\
	(:predicates                                 \n\
		(pred ?r)                                  \n\
	 )                                           \n\
	(:functions                                  \n\
	 (func ?r)                                   \n\
	)                                            \n\
	(:action test-action                         \n\
	 :parameters (?t)                            \n\
	 :precondition (and (pred ?t)                \n\
									 (= (func ?t) 1.1))          \n\
	 :effect (increase (pred ?t) (pred ?t))      \n\
	 )                                           \n\
)");
		SUCCEED() << " Function domain parsed ";
	} catch (PddlParserException const &e) {
		FAIL() << " Unexpected Exception: " << e.what();
	}
}
TEST(PddlParserTest, IPC2014)
{
	using recursive_directory_iterator = std::filesystem::recursive_directory_iterator;
	std::vector<std::string> domains;
	std::string              domain_suffix = "domain.pddl";
	for (const auto &dir : recursive_directory_iterator(std::string(SRCDIR) + "/ipc2014")) {
		std::string dirEntry    = dir.path().stem().string();
		std::string file_ending = dir.path().extension().string();
		if (file_ending == ".pddl" && dirEntry.find("domain") != std::string::npos) {
			domains.push_back(dir.path().string());
		}
	}
	for (const auto &s : domains) {
		try {
			std::ifstream t(s);
			if (t.fail()) {
				FAIL() << " Failed to read file: " << s;
			}
			std::stringstream buffer;
			buffer << t.rdbuf();
			PddlParser p;
			p.parseDomain(buffer.str());
		} catch (PddlParserException const &e) {
			FAIL() << " Unexpected Exception in file:\n" << s << "\n" << e.what();
		}
		SUCCEED() << " All files parsed without failure.";
	}
}
