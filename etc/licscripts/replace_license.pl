#!/usr/bin/perl

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

