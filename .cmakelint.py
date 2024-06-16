# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Library General Public License for more details.
#
# Read the full text in the LICENSE.GPL file in the doc directory.
# ----------------------------
# Options affecting the linter
# ----------------------------
with section("lint"):
    # C0307 complains after indentation after formatting
    # C0301 is the 80 character limit, but we do not enforce it,
    #       if the formatter is happy, so are we.
    # R0912 and R0915 enforce the max number of statements and
    #       if-statements per scope
    disabled_codes = ["R0912", "R0915"]
    # ["C0111", "C0307", "R0912", "R0915", "C0301", "C0103", "C0303"]
