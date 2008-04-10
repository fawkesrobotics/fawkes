#!/usr/bin/perl

if ( scalar(@ARGV) < 2 ) {
  print("Insufficient number of arguments.\n");
  print("Usage: finclic.pl <file_to_search> <license_file> [license_file...]\n\n");
}

$FILE=$ARGV[0];
@LIC_FILES = ();
for ($i = 1; $i < scalar(@ARGV); $i++) {
  push(@LIC_FILES, $ARGV[$i]);
}

foreach $licfile (@LIC_FILES) {
  open(SEARCH, $licfile);
  open(WORK, $FILE);

  undef $/;
  $search = <SEARCH>;
  $work = <WORK>;

  close(SEARCH);
  close(WORK);

  if ( $work =~ /\Q$search/s ) {
    printf("++ File %s did match %s\n", $FILE, $licfile);
    exit(0);
  }
}

printf("** File %s did NOT match any license\n", $FILE);
exit(1);

