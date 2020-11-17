/***************************************************************************
 *  uuid.cpp - uuid_t wrapper
 *
 *  Created: Tue 17 Nov 2020 10:17:15 CET 10:17
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

#include "core/exceptions/software.h"

#include <utils/uuid.h>
#include <uuid/uuid.h>

namespace fawkes {

Uuid::Uuid() noexcept
{
	uuid_generate(uuid_);
}

Uuid::~Uuid() noexcept
{
	uuid_clear(uuid_);
}

Uuid::Uuid(const Uuid &other) noexcept
{
	uuid_copy(uuid_, other.uuid_);
}

Uuid::Uuid(Uuid &&other) noexcept
{
	uuid_copy(uuid_, other.uuid_);
	uuid_clear(other.uuid_);
}

Uuid::Uuid(const char *string)
{
	int res = uuid_parse(string, uuid_);
	if (res != 0) {
		throw IllegalArgumentException("Cannot parse '%s' into a uuid", string);
	}
}

Uuid &
Uuid::operator=(const Uuid &other) noexcept
{
	uuid_copy(uuid_, other.uuid_);
	return *this;
}

Uuid &
Uuid::operator=(Uuid &&other) noexcept
{
	uuid_copy(uuid_, other.uuid_);
	uuid_clear(other.uuid_);
	return *this;
}

std::string
Uuid::get_string() const
{
	char res[37];
	uuid_unparse(uuid_, res);
	return std::string(res);
}

bool
operator<(const Uuid &uuid, const Uuid &other) noexcept
{
	return (uuid_compare(uuid.uuid_, other.uuid_) < 0);
}

bool
operator==(const Uuid &uuid, const Uuid &other) noexcept
{
	return (uuid_compare(uuid.uuid_, other.uuid_) == 0);
}

bool
operator!=(const Uuid &uuid, const Uuid &other) noexcept
{
	return (uuid_compare(uuid.uuid_, other.uuid_) != 0);
}

} // namespace fawkes
