#!/usr/bin/perl

# Copyright (C) 2021 Till Hofmann <hofmann@kbsg.rwth-aachen.de>
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>.

@LIC_SEARCH = ("license.search");
$LIC_REPLACE = "license.gpl";
$FILE = $ARGV[0];

foreach $licfile (@LIC_SEARCH) {

  open(SEARCH, $licfile);
  open(REPLACE, $LIC_REPLACE);
  open(WORK, $FILE);

  undef $/;
  $search = <SEARCH>;
  $replace = <REPLACE>;
  $work = <WORK>;

  # Search only
  #if ( $work =~ /\Q$search/s ) {
  # Replace
  if ( $work =~ s/\Q$search/$replace/s ) {
    close(WORK);
    open(WORK, "> $FILE");
    print WORK $work;
    printf("-- File %s matched, replaced license (%s)\n", $FILE, $licfile);
    exit(0);
  }
}

printf("** File %s did NOT match, control\n", $ARGV[2]);
exit(1);
