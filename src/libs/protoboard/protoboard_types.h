
/***************************************************************************
 * Protoboard plugin template
 * - General forward declarations
 *
 * Copyright 2019 Victor Mataré
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

#ifndef PROTOBOARD_TYPES_H_
#define PROTOBOARD_TYPES_H_

#include <boost/bimap.hpp>
#include <memory>
#include <unordered_map>

namespace protoboard {

class pb_convert;

typedef std::unordered_map<std::string, std::shared_ptr<pb_convert>> pb_conversion_map;

template <typename... Ts>
struct type_list
{
};

class BlackboardManager;

template <class IfaceT>
struct InterfaceMessageHandler;

template <class pbEnumT, class bbEnumT>
class enum_map
{
public:
	typedef boost::bimap<pbEnumT, bbEnumT> bimapT;
	constexpr enum_map(std::initializer_list<typename bimapT::value_type> init)
	: list(init), map(list.begin(), list.end())
	{
	}

	constexpr bbEnumT
	of(pbEnumT v) const
	{
		return map.left.at(v);
	}

	constexpr pbEnumT
	of(bbEnumT v) const
	{
		return map.right.at(v);
	}

private:
	const std::vector<typename bimapT::value_type> list;
	const bimapT                                   map;
};

} // namespace protoboard

#endif // PROTOBOARD_TYPES_H_
