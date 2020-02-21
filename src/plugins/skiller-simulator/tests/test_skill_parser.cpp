/***************************************************************************
 *  test_skill_parser.cpp - Tests for ExecutionTimeEstimator::Skill
 *
 *  Created: Sun 22 Dec 2019 19:23:47 CET 19:23
 *  Copyright  2019  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include "../execution_time_estimator.h"

#include <gtest/gtest.h>

using Skill = fawkes::skiller_simulator::ExecutionTimeEstimator::Skill;

TEST(SkillParserTest, EmptySkill)
{
	Skill s{""};
	ASSERT_EQ(s.skill_name, "");
	ASSERT_EQ(s.skill_args.size(), 0);
}

TEST(SkillParserTest, SkillWithoutArgs)
{
	Skill s{"say()"};
	ASSERT_EQ(s.skill_name, "say");
	ASSERT_EQ(s.skill_args.size(), 0);
}

TEST(SkillParserTest, SkillWithIntArg)
{
	Skill s{"count{i=1}"};
	ASSERT_EQ(s.skill_name, "count");
	ASSERT_EQ(s.skill_args.size(), 1);
	ASSERT_EQ(s.skill_args["i"], "1");
}

TEST(SkillParserTest, SkillWithStringArg)
{
	Skill s("say{text='hello'}");
	ASSERT_EQ(s.skill_name, "say");
	ASSERT_EQ(s.skill_args.size(), 1);
	ASSERT_EQ(s.skill_args["text"], "hello");
	s = Skill("say{text=\"hello\"}");
	ASSERT_EQ(s.skill_name, "say");
	ASSERT_EQ(s.skill_args.size(), 1);
	ASSERT_EQ(s.skill_args["text"], "hello");
}

TEST(SkillParserTest, SkillWithMultipleArgs)
{
	Skill s("say{text='hello', second='bye'}");
	ASSERT_EQ(s.skill_name, "say");
	ASSERT_EQ(s.skill_args.size(), 2);
	ASSERT_EQ(s.skill_args["text"], "hello");
	ASSERT_EQ(s.skill_args["second"], "bye");
}

TEST(SkillParserTest, SkillWithNewline)
{
	Skill s("say{text=\n\"hello\"}");
	ASSERT_EQ(s.skill_name, "say");
	ASSERT_EQ(s.skill_args.size(), 1);
	ASSERT_EQ(s.skill_args["text"], "hello");
}
