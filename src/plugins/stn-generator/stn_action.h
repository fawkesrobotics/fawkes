
/***************************************************************************
 *  stn_action.h - stn-generator
 *
 *  Created: Sat May  6 20:16:21 2017
 *  Copyright  2017  Matthias Loebach
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

#ifndef PLUGINS_STN_ACTION_H_
#define PLUGINS_STN_ACTION_H_

#include "predicate.h"

#include <algorithm>
#include <atomic>
#include <iostream>
#include <iterator>
#include <map>
#include <string>
#include <vector>

namespace fawkes {
namespace stn {

enum EdgeType { CONDITIONAL, TEMPORAL };

class StnAction
{
public:
	StnAction(const std::string &             name,
	          const std::vector<Predicate> &  preconds,
	          const std::vector<Predicate> &  effects,
	          const std::string &             opts,
	          size_t                          duration      = 0,
	          const std::vector<std::string> &cond_breakups = {},
	          const std::vector<std::string> &temp_breakups = {});
	StnAction(){};
	virtual ~StnAction(){};

	bool operator==(const StnAction &o);
	bool operator!=(const StnAction &o);

	size_t                        id() const;
	bool                          checkForBreakup(EdgeType t, const Predicate &p) const;
	std::vector<size_t>           condActionIds() const;
	std::string                   genGraphNodeName() const;
	std::string                   genConditionEdgeLabel(size_t cond_action) const;
	std::string                   genTemporalEdgeLabel() const;
	void                          genConditionalActions(std::vector<StnAction> candidate_actions);
	const std::vector<Predicate> &effects() const;
	std::string                   name() const;
	size_t                        duration() const;
	std::string                   opts() const;

private:
	friend std::ostream &    operator<<(std::ostream &, const StnAction &);
	size_t                   id_;
	std::string              name_;
	std::vector<Predicate>   preconds_;
	std::vector<Predicate>   effects_;
	std::string              opts_;
	size_t                   duration_;
	std::vector<std::string> cond_breakups_;
	std::vector<std::string> temp_breakups_;
	std::map<size_t, std::pair<std::string, std::vector<Predicate>>> cond_actions_;
	static size_t                                                    count;
};
} // namespace stn
} // namespace fawkes
#endif
