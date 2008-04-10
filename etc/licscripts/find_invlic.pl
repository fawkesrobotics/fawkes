#!/usr/bin/perl

use File::Find;
use Fcntl ':mode';

if ( scalar(@ARGV) < 2 ) {
  print("Insufficient number of arguments.\n");
  print("Usage: finclic.pl <dir_to_search> <license_file> [license_file...]\n\n");
}

$DIRECTORY=shift(@ARGV);
@LICENSES = ();
undef $/;

foreach $f (@ARGV) {
  open(LIC, $f) || die "Could not open license file $f";
  $lic = <LIC>;
  close(LIC);
  push(@LICENSES, $lic);
}

$ok = 1;

#sub check_dir()
#{
#  local $dir = shift;
#  local @subdirs = ();

#}

#opendir(DIR, $DIRECTORYI);

find(\&check_file, $DIRECTORY);

sub check_file()
{
  local $entry = $_;

  # Ignore directories
  local @s = stat($entry);
  if ( S_ISDIR($s[2]) ) {
    if ( ($entry =~ /^\./ || $entry eq "extlib") && $entry ne "." ) {
      $File::Find::prune = 1;
    }
    return;
  }

  # Only operate on .h, .c and .cpp files
  if ( $entry !~ /\.(cpp|h|c)$/ ) {
    return;
  }

  open(WORK, $entry);
  local $work = <WORK>;
  close(WORK);

  foreach $lic (@LICENSES) {
    if ( $work =~ /\Q$lic/s ) {
      # printf("++ File %s did match\n", $entry);
      return;
    }
  }

  printf("** File %s did NOT match any license\n", $File::Find::dir . "/" . $entry);
  $ok = 0;
}

exit($ok ? 0 : 1);

