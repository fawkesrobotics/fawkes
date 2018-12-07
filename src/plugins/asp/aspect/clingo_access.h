/***************************************************************************
 *  clingo_access.h - Clingo access wrapper for the aspects
 *
 *  Created: Mon Oct 31 13:41:07 2016
 *  Copyright  2016 Björn Schäpers
 *             2018 Tim Niemueller [www.niemueller.org]
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

#ifndef _PLUGINS_ASP_ASPECT_CLINGO_ACCESS_H_
#define _PLUGINS_ASP_ASPECT_CLINGO_ACCESS_H_

#include <atomic>
#include <clingo.hh>
#include <functional>
#include <memory>
#include <vector>

#include <core/threading/mutex.h>

namespace fawkes {

class Logger;

class ClingoAccess : public Clingo::SolveEventHandler
{
public:
	/** Debug levels, higher levels include the lower values. */
	enum DebugLevel_t
	{
	 ASP_DBG_NONE = 0,                ///< No debug output at all.
	 ASP_DBG_TIME = 10,               ///< Print when starting/finishing grounding/solving for analysis.
	 ASP_DBG_PROGRAMS = 20,           ///< Print which programs are grounded.
	 ASP_DBG_EXTERNALS = 30,          ///< Print assignments and releases of externals.
	 ASP_DBG_MODELS = 40,             ///< Print new models.
	 ASP_DBG_ALL_MODEL_SYMBOLS = 50,  ///< Ignore '\#show' statements and print all symbols of a model.
	 ASP_DBG_ALL,                     ///< Print everything.
	 ASP_DBG_EVEN_CLINGO              ///< Activates the --output-debug=text option for clingo.
	};

	ClingoAccess(Logger *logger, const std::string& log_component);
	~ClingoAccess(void);

	void register_model_callback(std::shared_ptr<std::function<bool(void)>> callback);
	void unregister_model_callback(std::shared_ptr<std::function<bool(void)>> callback);
	void register_finish_callback(std::shared_ptr<std::function<void(Clingo::SolveResult)>> callback);
	void unregister_finish_callback(std::shared_ptr<std::function<void(Clingo::SolveResult)>> callback);

	void set_ground_callback(Clingo::GroundCallback&& callback);

	bool solving(void) const noexcept;
	bool start_solving(void);
	bool start_solving_blocking(void);
	bool cancel_solving(void);

	bool reset(void);

	void set_num_threads(const int threads, const bool use_splitting = false);
	int num_threads(void) const noexcept;

	Clingo::SymbolVector model_symbols(void) const;

	bool load_file(const std::string& path);

	bool ground(const Clingo::PartSpan& parts);

	inline bool
	assign_external(const Clingo::Symbol& atom, const bool value)
	{
		return assign_external(atom, value ? Clingo::TruthValue::True : Clingo::TruthValue::False);
	}

	inline bool
	free_external(const Clingo::Symbol& atom)
	{
		return assign_external(atom, Clingo::TruthValue::Free);
	}

	bool assign_external(const Clingo::Symbol& atom, const Clingo::TruthValue value);
	bool release_external(const Clingo::Symbol& atom);

	DebugLevel_t debug_level() const;
	void         set_debug_level(DebugLevel_t log_level);

private:
	bool on_model(Clingo::Model& model) override;
	void on_finish(Clingo::SolveResult result) override;

	void alloc_control(void);

private:
	Logger* const logger_;
	const std::string log_comp_;

	std::atomic<DebugLevel_t> debug_level_;

	int num_threads_;
	bool thread_mode_splitting_;

	Mutex control_mutex_;
	bool control_is_locked_;
	Clingo::Control *control_;

	mutable Mutex model_mutex_;
	Clingo::SymbolVector model_symbols_, old_symbols_;
	unsigned int model_counter_;

	std::atomic_bool solving_;
	mutable Mutex callback_mutex_;
	std::vector<std::shared_ptr<std::function<bool(void)>>> model_callbacks_;
	std::vector<std::shared_ptr<std::function<void(Clingo::SolveResult)>>> finish_callbacks_;
	Clingo::GroundCallback ground_callback_;
	Clingo::SolveHandle async_handle_;
};

} // end namespace fawkes

#endif
