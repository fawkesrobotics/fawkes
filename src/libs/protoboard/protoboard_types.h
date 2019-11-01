
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

/**
 * Helper structure to wrap a list of types into a single type.
 */
template <typename... Ts>
struct type_list
{
};

class BlackboardManager;

template <class IfaceT>
struct InterfaceMessageHandler;

/**
 * A compile-time constant bidirectional map that can be used to match blackboard interface enum
 * values to ProtoBuf's enum values
 * @tparam pbEnumT a ProtoBuf enum type
 * @tparam bbEnumT a blackboard interface enum type
 */
template <class pbEnumT, class bbEnumT>
class enum_map
{
private:
	typedef boost::bimap<pbEnumT, bbEnumT> bimapT;

public:
	/**
	 * constexpr constructor
	 * @param init A curly-brace initializer list that defines the entire mapping
	 */
	constexpr enum_map(std::initializer_list<typename bimapT::value_type> init)
	: list(init), map(list.begin(), list.end())
	{
	}

	/**
	 * @param v a ProtoBuf enum value
	 * @return the mapped blackboard interface enum value
	 */
	constexpr bbEnumT
	of(pbEnumT v) const
	{
		return map.left.at(v);
	}

	/**
	 * @param v a blackboard interface enum value
	 * @return the mapped ProtoBuf enum value
	 */
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
