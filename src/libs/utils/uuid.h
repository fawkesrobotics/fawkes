/***************************************************************************
 *  uuid.h - uuid_t wrapper
 *
 *  Created: Tue 17 Nov 2020 10:09:43 CET 10:09
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

#pragma once

#include <uuid/uuid.h>

#include <string>

namespace fawkes {
class Uuid
{
public:
	Uuid() noexcept;
	~Uuid() noexcept;
	Uuid(const Uuid &other) noexcept;
	Uuid(Uuid &&other) noexcept;
	explicit Uuid(const char *string);
	Uuid &      operator=(const Uuid &other) noexcept;
	Uuid &      operator=(Uuid &&other) noexcept;
	std::string get_string() const;

	friend bool operator<(const Uuid &uuid, const Uuid &other) noexcept;
	friend bool operator==(const Uuid &uuid, const Uuid &other) noexcept;
	friend bool operator!=(const Uuid &uuid, const Uuid &other) noexcept;

private:
	uuid_t uuid_;
};

} // namespace fawkes
