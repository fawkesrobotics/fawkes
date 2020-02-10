/***************************************************************************
 *  test_circular_buffer.cpp - CircularBuffer Unit Test
 *
 *  Created: Fri Aug 15 16:27:42 2014
 *  Copyright  2014  Till Hofmann
 *
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

#define CATCH_CONFIG_MAIN
#include <core/utils/circular_buffer.h>

#include <catch2/catch.hpp>
#include <stdexcept>

using namespace fawkes;

TEST_CASE("Access elements", "[circular_buffer]")
{
	CircularBuffer<int> buffer(1000);
	for (int i = 0; i < 1000; i++) {
		buffer.push_back(i);
	}
	for (int i = 0; i < 1000; i++) {
		REQUIRE(buffer[i] == i);
		REQUIRE(buffer.at(i) == i);
	}
}

TEST_CASE("Delete elements", "[circular_buffer]")
{
	CircularBuffer<int> buffer(1);
	buffer.push_back(1);
	buffer.push_back(2);
	REQUIRE(buffer.size() == 1);
	REQUIRE(buffer[0] == 2);
}

TEST_CASE("Out of max range", "[circular_buffer]")
{
	CircularBuffer<int> buffer(1);
	int                 i;
	CHECK_NOTHROW(i = buffer[1]);
	REQUIRE_THROWS_AS(i = buffer.at(1), std::out_of_range);
}

TEST_CASE("Out of range", "[circular_buffer]")
{
	CircularBuffer<int> buffer(2);
	buffer.push_back(1);
	int i;
	REQUIRE_NOTHROW(i = buffer[1]);
	REQUIRE_THROWS_AS(i = buffer.at(1), std::out_of_range);
}

TEST_CASE("Copy constructor", "[circular_buffer]")
{
	CircularBuffer<int> b1(5);
	b1.push_back(1);
	b1.push_back(2);
	CircularBuffer<int> b2(b1);
	REQUIRE(b2.get_max_size() == 5);
	REQUIRE(b2[0] == 1);
	REQUIRE(b2[1] == 2);
	b2.push_back(3);
	REQUIRE(b2[2] == 3);
	REQUIRE(b1[0] == 1);
	REQUIRE(b1[1] == 2);
	REQUIRE_THROWS_AS(b1.at(2), std::out_of_range);
}
