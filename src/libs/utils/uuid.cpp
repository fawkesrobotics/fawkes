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

/** @class Uuid
 * A convenience class for universally unique identifiers (UUIDs).
 * It wraps uuid(3) to allow easier creation and deletion of UUIDs.
 */

/** Generate a new Uuid. */
Uuid::Uuid() noexcept
{
	uuid_generate(uuid_);
}

/** Destructor, clears up the occupied storage. */
Uuid::~Uuid() noexcept
{
	uuid_clear(uuid_);
}

/** Copy constructor.
 * The newly constructed Uuid is guaranteed to be the same as the source Uuid.
 * @param other The Uuid to copy from
 */
Uuid::Uuid(const Uuid &other) noexcept
{
	uuid_copy(uuid_, other.uuid_);
}

/** Move constructor.
 * This leaves the other Uuid in an undefined state. The newly constructed Uuid
 * is guaranteed to be the same as the source Uuid.
 * @param other The Uuid to move from
 */
Uuid::Uuid(Uuid &&other) noexcept
{
	uuid_copy(uuid_, other.uuid_);
	uuid_clear(other.uuid_);
}

/** Construct a Uuid from a string.
 * @param string The string represenation of the Uuid, of the form
 * 1b4e28ba-2fa1-11d2-883f-b9a761bde3fb
 */
Uuid::Uuid(const char *string)
{
	int res = uuid_parse(string, uuid_);
	if (res != 0) {
		throw IllegalArgumentException("Cannot parse '%s' into a uuid", string);
	}
}

/** Assignment operator.
 * After assignment, both Uuids are guaranteed to be the same.
 * @param other The Uuid to assign from
 * @return A reference to the assigned Uuid
 */
Uuid &
Uuid::operator=(const Uuid &other) noexcept
{
	uuid_copy(uuid_, other.uuid_);
	return *this;
}

/** Move assignment operator.
 * This leaves the other Uuid in an undefined state. The assigned Uuid
 * is guaranteed to be the same as the source Uuid.
 * @param other The Uuid to assign from
 * @return A reference to the assigned Uuid
 */
Uuid &
Uuid::operator=(Uuid &&other) noexcept
{
	uuid_copy(uuid_, other.uuid_);
	uuid_clear(other.uuid_);
	return *this;
}

/** Get the string representation of the Uuid.
 * @return The Uuid as string of the form 1b4e28ba-2fa1-11d2-883f-b9a761bde3fb
 */
std::string
Uuid::get_string() const
{
	char res[37];
	uuid_unparse(uuid_, res);
	return std::string(res);
}

/** Compare two Uuids.
 * @param uuid The first Uuid to compare
 * @param other The second Uuid to compare
 * @return True if the first Uuid is smaller than the second
 */
bool
operator<(const Uuid &uuid, const Uuid &other) noexcept
{
	return (uuid_compare(uuid.uuid_, other.uuid_) < 0);
}

/** Compare two Uuids.
 * @param uuid The first Uuid to compare
 * @param other The second Uuid to compare
 * @return True if the two Uuids are the same
 */
bool
operator==(const Uuid &uuid, const Uuid &other) noexcept
{
	return (uuid_compare(uuid.uuid_, other.uuid_) == 0);
}

/** Compare two Uuids.
 * @param uuid The first Uuid to compare
 * @param other The second Uuid to compare
 * @return True if the two Uuids are not the same
 */
bool
operator!=(const Uuid &uuid, const Uuid &other) noexcept
{
	return (uuid_compare(uuid.uuid_, other.uuid_) != 0);
}

} // namespace fawkes
