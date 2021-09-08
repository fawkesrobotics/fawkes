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
	auto benchmarks = {R"delim(
(define (domain action-arg-pred-missmatch)
	(:requirements :strips :typing)
	(:types
		obj-a - object
		obj-b - object
	 )
	(:predicates
		(pred ?r - obj-a)
	 )
	(:action test-action
	 :parameters (?t - obj-b)
	 :precondition (pred ?t)
	 :effect (not (pred ?t))
	)
))delim",
	                   R"delim(
	(define (domain typing-disabled-but-type-defined)
	(:requirements :strips)
	(:types
		obj-a - object
	)
	(:predicates
		(pred ?r)
	)
	(:action test-action
	 :parameters (?t)
	 :precondition (pred ?t)
	 :effect (not (pred ?t))
	)
))delim",
	                   R"delim(
(define (domain typing-disabled-but-type-constant)
	(:requirements :strips)
	(:constants
		TEST_CONST - object
	 )
	(:predicates
		(pred ?r)
	 )
	(:action test-action
	 :parameters (?t)
	 :precondition (pred ?t)
	 :effect (not (pred ?t))
	)
))delim",
	                   R"delim(
(define (domain typing-disabled-but-type-param)
	(:requirements :strips)
	(:predicates
		(pred ?r)
	 )
	(:action test-action
	 :parameters (?t - object)
	 :precondition (pred ?t)
	 :effect (not (pred ?t))
	)
))delim",
	                   R"delim(
(define (domain action-unknown-type-in-pred)
	(:requirements :strips :typing)
	(:types
		obj-a - object
	 )
	(:predicates
		(pred ?r - obj-b)
	 )
	(:action test-action
	 :parameters (?t - obj-a)
	 :precondition (pred ?t)
	 :effect (not (pred ?t))
	)
))delim",
	                   R"delim(
(define (domain action-unknown-type-in-param)
	(:requirements :strips :typing)
	(:types
		obj-a - object
	 )
	(:predicates
		(pred ?r - obj-a)
	 )
	(:action test-action
	 :parameters (?t - obj-b)
	 :precondition (pred ?t)
	 :effect (not (pred ?t))
	)
))delim",
	                   R"delim(
(define (domain action-unknown-type-in-constant)
	(:requirements :strips :typing)
	(:types
		obj-a - object
	 )
	(:constants
		TEST_CONST - obj-b
	 )
	(:predicates
		(pred ?r - obj-a)
	 )
	(:action test-action
	 :parameters (?t - obj-a)
	 :precondition (pred ?t)
	 :effect (not (pred ?t))
	)
))delim",
	                   R"delim(
(define (domain action-constant-missmatch)
	(:requirements :strips :typing)
	(:types
		obj-a - object
		obj-b - object
	 )
	(:constants
		TEST_CONST - obj-b
	 )
	(:predicates
		(pred ?r - obj-a)
	 )
	(:action test-action
	 :parameters (?t - obj-a)
	 :precondition (pred TEST_CONST)
	 :effect (not (pred ?t))
	)
))delim"};
	for (const auto &s : benchmarks) {
		EXPECT_THROW(
		  {
			  PddlParser p;
			  p.parseDomain(s);
		  },
		  PddlTypeException);
	}
}

TEST(PddlParserTest, MinimalDomain)
{
	PddlParser p;
	Domain     d;
	EXPECT_NO_THROW(d = p.parseDomain(R"delim(
(define (domain test-domain)
	(:requirements)
	(:predicates
		(pred)
	 )
	(:action test-action
	 :parameters (?t)
	 :precondition (pred)
	 :effect (not (pred))
	)
))delim"););

	ASSERT_EQ(d.requirements.size(), 0);
	ASSERT_EQ(d.types.size(), 0);
	ASSERT_EQ(d.constants.size(), 0);
	ASSERT_EQ(d.predicates.size(), 1);
	ASSERT_EQ(d.functions.size(), 0);
	ASSERT_EQ(d.actions.size(), 1);
	ASSERT_EQ(d.actions[0].name, "test-action");
	ASSERT_EQ(d.actions[0].action_params.size(), 1);
	// action precondition correctly parsed?
	ASSERT_EQ(d.actions[0].precondition.type, ExpressionType::PREDICATE);
	ASSERT_EQ(boost::get<Predicate>(d.actions[0].precondition.expression).function, "pred");
	ASSERT_EQ(boost::get<Predicate>(d.actions[0].precondition.expression).arguments.size(), 0);
	// action effect correctly parsed?
	ASSERT_EQ(d.actions[0].effect.type, ExpressionType::BOOL);
	ASSERT_EQ(boost::get<Predicate>(d.actions[0].effect.expression).function, "not");
	ASSERT_EQ(boost::get<Predicate>(d.actions[0].effect.expression).arguments.size(), 1);
	ASSERT_EQ(boost::get<Predicate>(d.actions[0].effect.expression).arguments[0].type,
	          ExpressionType::PREDICATE);
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(d.actions[0].effect.expression).arguments[0].expression)
	            .function,
	          "pred");
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(d.actions[0].effect.expression).arguments[0].expression)
	            .arguments.size(),
	          0);
}

TEST(PddlParserTest, DurativeAction)
{
	PddlParser p;
	Domain     d;
	EXPECT_NO_THROW(d = p.parseDomain(R"delim(
(define (domain test-durative-action)
	(:requirements :strips :durative-actions)
	(:predicates
		(pred ?r)
	)
	(:durative-action test-action
	 :parameters (?t)
	 :duration (= ?duration 5.5)
	 :condition (and (at start (pred ?t))
		               (over all (pred ?t)))
	 :effect (and (at end (not (pred ?t)))
	              (at start (pred ?t)))
	)
))delim"););
	ASSERT_EQ(d.requirements.size(), 2);
	ASSERT_EQ(d.types.size(), 0);
	ASSERT_EQ(d.constants.size(), 0);
	ASSERT_EQ(d.predicates.size(), 1);
	ASSERT_EQ(d.functions.size(), 0);
	ASSERT_EQ(d.actions.size(), 1);
	ASSERT_EQ(d.actions[0].name, "test-action");
	ASSERT_EQ(d.actions[0].duration.type, ExpressionType::VALUE);
	ASSERT_EQ(boost::get<Atom>(d.actions[0].duration.expression), "5.5");
	ASSERT_EQ(d.actions[0].action_params.size(), 1);
	// action precondition correctly parsed?
	ASSERT_EQ(d.actions[0].precondition.type, ExpressionType::BOOL);
	ASSERT_EQ(boost::get<Predicate>(d.actions[0].precondition.expression).function, "and");
	ASSERT_EQ(boost::get<Predicate>(d.actions[0].precondition.expression).arguments.size(), 2);
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(d.actions[0].precondition.expression).arguments[0].expression)
	            .function,
	          "at start");
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(d.actions[0].precondition.expression).arguments[0].expression)
	            .arguments.size(),
	          1);
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(d.actions[0].precondition.expression).arguments[0].expression)
	            .arguments[0]
	            .type,
	          ExpressionType::PREDICATE);
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(
	              boost::get<Predicate>(d.actions[0].precondition.expression).arguments[0].expression)
	              .arguments[0]
	              .expression)
	            .function,
	          "pred");
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(
	              boost::get<Predicate>(d.actions[0].precondition.expression).arguments[0].expression)
	              .arguments[0]
	              .expression)
	            .arguments.size(),
	          1);
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(
	              boost::get<Predicate>(d.actions[0].precondition.expression).arguments[0].expression)
	              .arguments[0]
	              .expression)
	            .arguments[0]
	            .type,
	          ExpressionType::ATOM);
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(d.actions[0].precondition.expression).arguments[1].expression)
	            .function,
	          "over all");
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(d.actions[0].precondition.expression).arguments[1].expression)
	            .arguments.size(),
	          1);
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(d.actions[0].precondition.expression).arguments[1].expression)
	            .arguments[0]
	            .type,
	          ExpressionType::PREDICATE);
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(
	              boost::get<Predicate>(d.actions[0].precondition.expression).arguments[1].expression)
	              .arguments[0]
	              .expression)
	            .function,
	          "pred");
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(
	              boost::get<Predicate>(d.actions[0].precondition.expression).arguments[1].expression)
	              .arguments[0]
	              .expression)
	            .arguments.size(),
	          1);
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(
	              boost::get<Predicate>(d.actions[0].precondition.expression).arguments[1].expression)
	              .arguments[0]
	              .expression)
	            .arguments[0]
	            .type,
	          ExpressionType::ATOM);
	// action effect correctly parsed?
	ASSERT_EQ(boost::get<Predicate>(d.actions[0].effect.expression).function, "and");
	ASSERT_EQ(boost::get<Predicate>(d.actions[0].effect.expression).arguments.size(), 2);
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(d.actions[0].effect.expression).arguments[0].expression)
	            .function,
	          "at end");
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(d.actions[0].effect.expression).arguments[0].expression)
	            .arguments.size(),
	          1);
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(d.actions[0].effect.expression).arguments[0].expression)
	            .arguments[0]
	            .type,
	          ExpressionType::BOOL);
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(
	              boost::get<Predicate>(d.actions[0].effect.expression).arguments[0].expression)
	              .arguments[0]
	              .expression)
	            .function,
	          "not");
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(
	              boost::get<Predicate>(d.actions[0].effect.expression).arguments[0].expression)
	              .arguments[0]
	              .expression)
	            .arguments.size(),
	          1);
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(
	              boost::get<Predicate>(d.actions[0].effect.expression).arguments[0].expression)
	              .arguments[0]
	              .expression)
	            .arguments[0]
	            .type,
	          ExpressionType::PREDICATE);
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(
	              boost::get<Predicate>(
	                boost::get<Predicate>(d.actions[0].effect.expression).arguments[0].expression)
	                .arguments[0]
	                .expression)
	              .arguments[0]
	              .expression)
	            .function,
	          "pred");
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(
	              boost::get<Predicate>(
	                boost::get<Predicate>(d.actions[0].effect.expression).arguments[0].expression)
	                .arguments[0]
	                .expression)
	              .arguments[0]
	              .expression)
	            .arguments.size(),
	          1);
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(
	              boost::get<Predicate>(
	                boost::get<Predicate>(d.actions[0].effect.expression).arguments[0].expression)
	                .arguments[0]
	                .expression)
	              .arguments[0]
	              .expression)
	            .arguments[0]
	            .type,
	          ExpressionType::ATOM);
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(d.actions[0].effect.expression).arguments[1].expression)
	            .function,
	          "at start");
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(d.actions[0].effect.expression).arguments[1].expression)
	            .arguments.size(),
	          1);
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(d.actions[0].effect.expression).arguments[1].expression)
	            .arguments[0]
	            .type,
	          ExpressionType::PREDICATE);
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(
	              boost::get<Predicate>(d.actions[0].effect.expression).arguments[1].expression)
	              .arguments[0]
	              .expression)
	            .function,
	          "pred");
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(
	              boost::get<Predicate>(d.actions[0].effect.expression).arguments[1].expression)
	              .arguments[0]
	              .expression)
	            .arguments.size(),
	          1);
	ASSERT_EQ(boost::get<Predicate>(
	            boost::get<Predicate>(
	              boost::get<Predicate>(d.actions[0].effect.expression).arguments[1].expression)
	              .arguments[0]
	              .expression)
	            .arguments[0]
	            .type,
	          ExpressionType::ATOM);
}

TEST(PddlParserTest, Functions)
{
	EXPECT_NO_THROW(PddlParser p; p.parseDomain(R"delim(
(define (domain test-functions)
	(:requirements :strips :numeric-fluents)
	(:predicates
		(pred ?r)
	 )
	(:functions
	 (func ?r)
	)
	(:action test-action
	 :parameters (?t)
	 :precondition (and (pred ?t)
	                    (= (func ?t) 1.1))
	 :effect (increase (pred ?t) (pred ?t))
	 )
))delim"););
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
		EXPECT_NO_THROW(std::ifstream t(s); if (t.fail()) {
			FAIL() << " Failed to read file: " << s;
		} std::stringstream buffer;
		                buffer << t.rdbuf();
		                PddlParser p;
		                p.parseDomain(buffer.str()););
	}
}
