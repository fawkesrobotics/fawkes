
/***************************************************************************
 *  filepath.ecl - Utilities used to locate files in a configured path
 *
 *  Created: Thu Oct 09 19:34:47 2014
 *  Copyright  2014  Tim Niemueller
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

:- module(filepath).

:- export locate_file/2.
:- export locate_module/2.
:- export add_library_path/1.

:- external(locate_file/2, p_locate_file).

locate_module(Module, File) :-
    get_flag(prolog_suffix, X), !,
    locate_module_suffixes(Module, X, File).


locate_module_suffix(Module, X, File) :-
    append_strings(Module, X, ModuleWithSuffix),
    locate_file(ModuleWithSuffix, File).


locate_module_suffixes(_, [], _) :- !, fail.

locate_module_suffixes(Module, [X|Y], File) :-
    locate_module_suffix(Module, X, File); !, locate_module_suffixes(Module, Y, File).

add_library_path(P) :-
	get_flag(library_path, CurPath),
	( P = [_|_] ->
		append(CurPath, P, NewPath)
	;
		append(CurPath, [P], NewPath)
	),
	set_flag(library_path, NewPath).
