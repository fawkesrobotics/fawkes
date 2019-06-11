#!/usr/bin/perl

use File::Find;
use File::Spec;
use Fcntl ':mode';
use Getopt::Long;
use List::Util qw(any);

my @known_files = ();
my $known_files_file;

my @paths = ();
GetOptions("p=s" => \@paths, "k=s" => \$known_files_file);

$use_known_files = defined $known_files_file;

if ($use_known_files) {
  open(my $fh, '<', $known_files_file) or die "Could not open file with git-known files";
  chomp(@known_files = <$fh>);
  close($fh);
}

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
  if ( ! $use_known_files or any { $_ =~ File::Spec->rel2abs($entry) } @known_files) { 
    #if ( ! $use_known_files or File::Spec->rel2abs($entry) ~~ known_files) { 
    printf("** File %s did NOT match any license\n", $File::Find::dir . "/" . $entry);
    $ok = 0;
  } else {
    printf("** File %s did NOT match any license, but is not known to git, ignored!\n", $File::Find::dir . "/" . $entry);
  }
}

exit($ok ? 0 : 1);

