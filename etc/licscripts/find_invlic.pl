#!/usr/bin/perl

use File::Find;
use Fcntl ':mode';
use Getopt::Long;

my @paths = ();
GetOptions("p=s" => \@paths);

if ( scalar(@ARGV) == 0 ) {
  print("Insufficient number of arguments.\n");
  print("Usage: finclic.pl -p <dir_to_search> [-p ...] <license_file> [license_file...]\n\n");
}

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

foreach $p (@paths) {
  find(\&check_file, $p);
}

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

  # Only operate on certain files
  if ( $File::Find::dir =~ /\/node_modules\//) {
    return;
  }
  if ( $entry !~ /\.(cpp|h|c|cxx|hpp|ext_h|lua|py|ts)$/ ) {
    return;
  }
  if ( $entry =~ /\.(pb.cpp|pb.h)$/ ) {
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

