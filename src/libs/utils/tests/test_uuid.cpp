/***************************************************************************
 *  test_uuid.cpp -
 *
 *  Created: Tue 17 Nov 2020 15:09:33 CET 15:09
 *  Copyright  2020  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include <utils/uuid.h>

#include <catch2/catch.hpp>

using fawkes::Uuid;

TEST_CASE("Generate UUID", "[uuid]")
{
	Uuid uuid1;
	REQUIRE(uuid1.get_string() != "");
}

TEST_CASE("UUIDs are unique", "[uuid]")
{
	Uuid uuid1;
	Uuid uuid2;
	REQUIRE(uuid1 != uuid2);
}

TEST_CASE("Copy constructor", "[uuid]")
{
	Uuid uuid1;
	Uuid uuid2(uuid1);
	REQUIRE(uuid1 == uuid2);
}

TEST_CASE("Move constructor", "[uuid]")
{
	Uuid        uuid1;
	std::string uuid_string = uuid1.get_string();
	Uuid        uuid2{std::move(uuid1)};
	REQUIRE(uuid2.get_string() == uuid_string);
}

TEST_CASE("Create from string", "[uuid]")
{
	Uuid uuid1;
	Uuid uuid2{uuid1.get_string().c_str()};
	REQUIRE(uuid1 == uuid2);
}

TEST_CASE("Copy assignment", "[uuid]")
{
	Uuid uuid1;
	Uuid uuid2;
	uuid2 = uuid1;
	REQUIRE(uuid1 == uuid2);
}

TEST_CASE("Move assignment", "[uuid]")
{
	Uuid        uuid1;
	Uuid        uuid2;
	std::string uuid_string = uuid1.get_string();
	uuid2                   = std::move(uuid1);
	REQUIRE(uuid2.get_string() == uuid_string);
}
