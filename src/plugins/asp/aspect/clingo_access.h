/***************************************************************************
 *  clingo_access.h - Clingo access wrapper for the aspects
 *
 *  Created: Mon Oct 31 13:41:07 2016
 *  Copyright  2016 Björn Schäpers
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __PLUGINS_ASP_ASPECT_CLINGO_ACCESS_H_
#define __PLUGINS_ASP_ASPECT_CLINGO_ACCESS_H_

#include <atomic>
#include <clingo.hh>
#include <functional>
#include <memory>
#include <vector>

#include <core/threading/mutex.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Logger;

class ClingoAccess
{
	private:
	Logger* const Log;
	const std::string LogComponent;

	mutable Mutex ControlMutex;
	Clingo::Control Control;
	Clingo::SymbolVector ModelSymbols, OldSymbols;

	std::atomic_bool Solving;
	mutable Mutex CallbackMutex;
	std::vector<std::shared_ptr<std::function<bool(void)>>> ModelCallbacks;
	std::vector<std::shared_ptr<std::function<void(Clingo::SolveResult)>>> FinishCallbacks;

	bool newModel(const Clingo::Model& model);
	void solvingFinished(const Clingo::SolveResult result);

	public:
	std::atomic_bool Debug;

	ClingoAccess(Logger *log, const std::string& logComponent);

	void registerModelCallback(std::shared_ptr<std::function<bool(void)>> callback);
	void unregisterModelCallback(std::shared_ptr<std::function<bool(void)>> callback);
	void registerFinishCallback(std::shared_ptr<std::function<void(Clingo::SolveResult)>> callback);
	void unregisterFinishCallback(std::shared_ptr<std::function<void(Clingo::SolveResult)>> callback);

	bool solving(void) const noexcept;
	bool startSolving(void);
	bool startSolvingBlocking(void);
	bool cancelSolving(void);

	Clingo::SymbolVector modelSymbols(void) const;

	bool loadFile(const std::string& path);

	bool ground(const Clingo::PartSpan& parts);

	bool assign_external(const Clingo::Symbol atom, const Clingo::TruthValue value);
	bool release_external(const Clingo::Symbol atom);
};

} // end namespace fawkes

#endif
